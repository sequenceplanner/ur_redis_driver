use crate::{RobotCommand, UR_DRIVER_SOCKET_PORT};

pub fn generate_core_script_from_template(
    robot_name: &str,
    robot_command: RobotCommand,
    templates: &tera::Tera,
) -> Result<String, Box<dyn std::error::Error>> {
    let empty_context = tera::Context::new();
    match templates.render(
        &format!("{}.script", robot_command.command_type.to_string()),
        match &tera::Context::from_serialize(robot_command.clone()) {
            Ok(context) => context,
            Err(e) => {
                log::error!(
                    target: &format!("{}_ur_controller", robot_name),
                    "Creating a Tera Context from a serialized Interpretation failed with: {e}."
                );
                log::error!(
                    target: &format!("{}_ur_controller", robot_name),
                    "An empty Tera Context will be used instead."
                );
                &empty_context
            }
        },
    ) {
        Ok(script) => Ok(script),
        Err(e) => {
            log::error!(
                target: &format!("{}_ur_controller", robot_name),
                "Rendering the {}.script Tera Template failed with: {}.",
                robot_command.command_type,
                e
            );
            return Err(Box::new(e));
        }
    }
}

pub fn generate_ur_script(original: &str, host_address: &str) -> String {
    let indented_script: String = original
        .lines()
        .map(|l| format!("  {l}"))
        .collect::<Vec<String>>()
        .join("\n");

    let pre_script = r#"
def run_script():
"#
    .to_string();

    let post_script_1 = r#"

  def handshake():
"#
    .to_string();

    let post_script_2 = format!(
        "    socket_open(\"{}\", {}, \"ur_driver_socket\")",
        host_address, UR_DRIVER_SOCKET_PORT
    );

    let post_script_3 = r#"
    line_from_server = socket_read_line("ur_driver_socket", timeout=1.0)
    if(str_empty(line_from_server)):
      return False
    else:
      socket_send_line(line_from_server, "ur_driver_socket")
      return True
    end
  end

  if(handshake()):
    result = script()
    if(result):
      socket_send_line("ok", "ur_driver_socket")
    else:
      socket_send_line("error", "ur_driver_socket")
    end
  else:
"#
    .to_string();
    let post_script_4 = format!(
        "    popup(\"handshake failure with host {}, not moving.\")",
        host_address
    );

    let post_script_5 = r#"
  end

  socket_close("ur_driver_socket")
end

run_script()
"#
    .to_string();

    return format!(
        "{}{}{}{}{}{}{}",
        pre_script,
        indented_script,
        post_script_1,
        post_script_2,
        post_script_3,
        post_script_4,
        post_script_5
    );
}
