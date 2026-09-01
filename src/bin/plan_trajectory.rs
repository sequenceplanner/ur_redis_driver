//! Offline trajectory planning.
//!
//! Plans a path once, writes it to `trajectories/<name>.json`, and optionally
//! renders the URScript it would produce so it can be read before it is run. The
//! driver then executes it by name with `<robot>_trajectory_id`.
//!
//! ```text
//! cargo run --bin plan_trajectory -- plan --input path.json --name pick_approach
//! cargo run --bin plan_trajectory -- list
//! cargo run --bin plan_trajectory -- show pick_approach
//! cargo run --bin plan_trajectory -- render pick_approach --script /tmp/out.script
//! ```
//!
//! The input file is a serialised `PlanRequest`. See `--help` for its shape.

use std::process::ExitCode;

use ur_redis_driver::trajectory::limits::PlanScaling;
use ur_redis_driver::trajectory::plan::{self, PlanRequest, PlannedTrajectory};
use ur_redis_driver::trajectory::{emit, store};

const USAGE: &str = r#"plan_trajectory - offline time-optimal trajectory planning for UR arms

USAGE:
    plan_trajectory plan   --input <FILE> --name <NAME> [OPTIONS]
    plan_trajectory list
    plan_trajectory show   <NAME>
    plan_trajectory render <NAME> [--script <FILE>] [--tcp <POSE>]

PLAN OPTIONS:
    --input <FILE>                 JSON plan request (see below)
    --name <NAME>                  Name to store it under; letters, digits, '_', '-', '.'
    --model <MODEL>                Override the request's robot_model, e.g. ur20
    --velocity-scaling <F>         (0.0, 1.0]; anything else means no scaling
    --acceleration-scaling <F>     (0.0, 1.0]; default 0.5, because the acceleration
                                   limits are estimates rather than published figures
    --acceleration-limits <a,...>  Six rad/s^2 values, overriding the estimates
    --script <FILE>                Also write the rendered URScript here
    --dry-run                      Plan and report, but do not store anything

ENVIRONMENT:
    UR_TRAJECTORY_DIR   Where trajectories are stored (default: trajectories/)
    ROBOT_MODEL         Fallback model when the request does not name one

INPUT FILE:
    {
      "robot_model": "ur20",
      "start": { "joints": [0, -1.5707, 0, -1.5707, 0, 0] },
      "waypoints": [
        { "target": { "joints": [0.8, -1.4, 0.5, -1.2, 0.3, 0.0] },
          "blend_radius": 0.10 },
        { "target": { "pose": { "values": [0.6, 0.1, 0.4, 0, 3.14, 0],
                                "frame": "urdf_base_link" } },
          "interpolation": "linear" }
      ],
      "scaling": { "velocity": 1.0, "acceleration": 0.5 },
      "max_path_deviation": 0.01,
      "max_cartesian_speed": 0.5
    }

    `frame` is "urdf_base_link" (the driver's default) or "ur_base".
    `interpolation` is "joint" (movej, the default) or "linear" (movel).
    Omitted fields take their defaults.
"#;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || args[0] == "-h" || args[0] == "--help" {
        print!("{USAGE}");
        return ExitCode::SUCCESS;
    }

    let result = match args[0].as_str() {
        "plan" => cmd_plan(&args[1..]),
        "list" => cmd_list(),
        "show" => cmd_show(&args[1..]),
        "render" => cmd_render(&args[1..]),
        other => Err(format!("unknown subcommand '{other}'; try --help")),
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Minimal `--flag value` parsing.
///
/// Hand-rolled rather than pulling in an argument-parsing crate for four flags;
/// this binary is a development tool, not the driver.
struct Args {
    flags: Vec<(String, Option<String>)>,
    positional: Vec<String>,
}

impl Args {
    fn parse(raw: &[String]) -> Args {
        let mut flags = Vec::new();
        let mut positional = Vec::new();
        let mut i = 0;
        while i < raw.len() {
            if let Some(name) = raw[i].strip_prefix("--") {
                let value = raw.get(i + 1).filter(|v| !v.starts_with("--")).cloned();
                if value.is_some() {
                    i += 1;
                }
                flags.push((name.to_string(), value));
            } else {
                positional.push(raw[i].clone());
            }
            i += 1;
        }
        Args { flags, positional }
    }

    fn get(&self, name: &str) -> Option<&str> {
        self.flags
            .iter()
            .find(|(n, _)| n == name)
            .and_then(|(_, v)| v.as_deref())
    }

    fn has(&self, name: &str) -> bool {
        self.flags.iter().any(|(n, _)| n == name)
    }

    fn require(&self, name: &str) -> Result<&str, String> {
        self.get(name).ok_or_else(|| format!("--{name} is required"))
    }

    fn float(&self, name: &str) -> Result<Option<f64>, String> {
        match self.get(name) {
            None => Ok(None),
            Some(v) => v
                .parse::<f64>()
                .map(Some)
                .map_err(|e| format!("--{name}: '{v}' is not a number: {e}")),
        }
    }
}

fn cmd_plan(raw: &[String]) -> Result<(), String> {
    let args = Args::parse(raw);
    let input = args.require("input")?;
    let name = args.require("name")?;
    store::validate_name(name)?;

    let text = std::fs::read_to_string(input).map_err(|e| format!("{input}: {e}"))?;
    let mut request: PlanRequest =
        serde_json::from_str(&text).map_err(|e| format!("{input} is not a plan request: {e}"))?;

    if let Some(model) = args.get("model") {
        request.robot_model = model.to_string();
    }
    if let Some(v) = args.float("velocity-scaling")? {
        request.scaling = PlanScaling { velocity: v, ..request.scaling };
    }
    if let Some(a) = args.float("acceleration-scaling")? {
        request.scaling = PlanScaling { acceleration: a, ..request.scaling };
    }
    if let Some(list) = args.get("acceleration-limits") {
        request.acceleration_max = Some(parse_six(list, "--acceleration-limits")?);
    }

    let trajectory = plan::plan(&request).map_err(|e| e.to_string())?;
    report(&trajectory);

    if let Some(path) = args.get("script") {
        let script = render(&trajectory, args.get("tcp"))?;
        std::fs::write(path, script).map_err(|e| format!("{path}: {e}"))?;
        println!("\nscript written to {path}");
    }

    if args.has("dry-run") {
        println!("\ndry run: nothing stored");
        return Ok(());
    }

    let path = store::save(name, &trajectory, Some(&request))?;
    println!("\nstored as {}", path.display());
    println!(
        "run it with: <robot>_command_type = \"{}\", <robot>_trajectory_id = \"{}\"",
        ur_redis_driver::OPTIMAL_TRAJECTORY_COMMAND,
        name
    );
    Ok(())
}

fn cmd_list() -> Result<(), String> {
    let names = store::list()?;
    if names.is_empty() {
        println!("no trajectories in {}", store::trajectory_dir().display());
        return Ok(());
    }
    for name in names {
        match store::load(&name) {
            Ok(s) => println!(
                "{:<28} {:>3} waypoints  {:>6.2} s  {}",
                name,
                s.trajectory.waypoints.len(),
                s.trajectory.duration_estimate,
                s.trajectory.robot_model
            ),
            Err(e) => println!("{name:<28} (unreadable: {e})"),
        }
    }
    Ok(())
}

fn cmd_show(raw: &[String]) -> Result<(), String> {
    let args = Args::parse(raw);
    let name = args
        .positional
        .first()
        .ok_or_else(|| "show needs a trajectory name".to_string())?;
    let stored = store::load(name)?;
    println!("{}  ({})", stored.name, stored.trajectory.robot_model);
    println!("stored at unix time {}", stored.created_unix_seconds);
    report(&stored.trajectory);
    Ok(())
}

fn cmd_render(raw: &[String]) -> Result<(), String> {
    let args = Args::parse(raw);
    let name = args
        .positional
        .first()
        .ok_or_else(|| "render needs a trajectory name".to_string())?;
    let stored = store::load(name)?;
    let script = render(&stored.trajectory, args.get("tcp"))?;
    match args.get("script") {
        Some(path) => {
            std::fs::write(path, &script).map_err(|e| format!("{path}: {e}"))?;
            println!("script written to {path}");
        }
        None => println!("{script}"),
    }
    Ok(())
}

/// Render the core script body, the way `command_server` would.
///
/// Only the body: the driver wraps it in the `run_script()` handshake at send time,
/// and that wrapper needs a host address that only makes sense on the machine
/// actually talking to the robot.
fn render(trajectory: &PlannedTrajectory, tcp: Option<&str>) -> Result<String, String> {
    let templates = tera::Tera::new("templates/*.script")
        .map_err(|e| format!("could not load templates/ (run from the repository root): {e}"))?;
    let command = emit::to_robot_command(
        trajectory,
        ur_redis_driver::OPTIMAL_TRAJECTORY_COMMAND,
        tcp.unwrap_or("p[0,0,0,0,0,0]"),
        "",
        true,
    );
    ur_redis_driver::generate_core_script_from_template("plan_trajectory", command, &templates)
        .map_err(|e| format!("rendering failed: {e}"))
}

fn report(t: &PlannedTrajectory) {
    println!(
        "{} waypoints, estimated {:.2} s (blending makes the real motion faster)",
        t.waypoints.len(),
        t.duration_estimate
    );
    println!(
        "\n{:>3}  {:>4}  {:>9}  {:>9}  {:>7}  {}",
        "#", "move", "velocity", "accel", "blend", "limited by"
    );
    for (i, w) in t.waypoints.iter().enumerate() {
        println!(
            "{:>3}  {:>4}  {:>9.4}  {:>9.4}  {:>7.4}  joint {}",
            i,
            if w.use_linear_motion { "movel" } else { "movej" },
            w.velocity,
            w.acceleration,
            w.blend_radius,
            w.limiting_joint
        );
    }
    if !t.warnings.is_empty() {
        println!();
        for warning in &t.warnings {
            println!("warning: {warning}");
        }
    }
}

fn parse_six(list: &str, flag: &str) -> Result<[f64; 6], String> {
    let parts: Vec<&str> = list.split(',').map(|s| s.trim()).collect();
    if parts.len() != 6 {
        return Err(format!("{flag} needs six comma-separated values, got {}", parts.len()));
    }
    let mut out = [0.0; 6];
    for (i, p) in parts.iter().enumerate() {
        out[i] = p
            .parse()
            .map_err(|e| format!("{flag} value {i} ('{p}') is not a number: {e}"))?;
    }
    Ok(out)
}
