//! Pins the analytic kinematics to the URDF the driver actually loads.
//!
//! The frame convention is the highest-risk part of the trajectory module: get it
//! wrong and the arm moves to a target mirrored about its own base. These tests
//! are the oracle, and `k::Chain`'s reading of `ur20.urdf` is the ground truth.

use k::nalgebra as na;
use ur_redis_driver::trajectory::frames::*;
use ur_redis_driver::trajectory::kinematics::*;

const URDF: &str = "src/ur_description/urdf/ur20.urdf";
const MODEL: &str = "ur20";

/// A fixed-seed linear congruential generator.
///
/// Deliberately not `rand`: a failing case must be reproducible from the seed
/// alone, with no dependency on which `rand` version resolved.
struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg(seed)
    }
    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn joints(&mut self) -> [f64; 6] {
        let mut q = [0.0; 6];
        for (i, v) in q.iter_mut().enumerate() {
            // The elbow is limited to +-pi; everything else to +-2pi. Stay just
            // inside so a clamp in `k` cannot quietly change the input.
            let lim = if i == 2 {
                std::f64::consts::PI
            } else {
                2.0 * std::f64::consts::PI
            };
            *v = self.range(-0.99 * lim, 0.99 * lim);
        }
        q
    }
}

fn chain() -> k::Chain<f64> {
    k::Chain::<f64>::from_urdf_file(URDF).expect("ur20.urdf parses")
}

/// The `tool0` node, looked up once.
///
/// The guard returned by `node.link()` **must be dropped before**
/// `node.world_transform()` is called: they contend for the same lock, and holding
/// the first across the second deadlocks. `state_publisher.rs` sidesteps this by
/// cloning the link name out and letting the guard fall before it reads any
/// transform; this does the same, and caching the node keeps it out of the hot loop
/// entirely.
fn tool0_node(chain: &k::Chain<f64>) -> k::Node<f64> {
    for node in chain.iter() {
        let name = node.link().as_ref().map(|l| l.name.clone());
        if name.as_deref() == Some("tool0") {
            return node.clone();
        }
    }
    panic!("ur20.urdf has no tool0 link");
}

/// FK from `base_link` to `tool0`, straight out of the URDF.
fn urdf_fk(chain: &k::Chain<f64>, tool0: &k::Node<f64>, q: &[f64; 6]) -> na::Isometry3<f64> {
    chain.set_joint_positions(q).expect("six joints");
    chain.update_link_transforms();
    tool0.world_transform().expect("tool0 has a world transform")
}

fn dh() -> DhParameters {
    DhParameters::for_model(MODEL).expect("ur20 is a known model")
}

fn pose_err(a: &na::Isometry3<f64>, b: &na::Isometry3<f64>) -> (f64, f64) {
    let dt = (a.translation.vector - b.translation.vector).norm();
    let dr = a.rotation.angle_to(&b.rotation);
    (dt, dr)
}

#[test]
fn urdf_exposes_the_six_joints_in_order() {
    let chain = chain();
    let names: Vec<String> = chain
        .iter_joints()
        .map(|j| j.name.clone())
        .collect();
    assert_eq!(
        names,
        vec![
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
        "joint order feeds set_joint_positions; a change here silently permutes FK"
    );
}

/// The load-bearing test: analytic FK, rotated into `base_link`, is the URDF's FK.
///
/// This is what proves `base` and `base_link` differ by exactly `Rz(pi)` and that
/// DH position 6 is `tool0`.
#[test]
fn analytic_fk_matches_urdf_fk_through_rz_pi() {
    let chain = chain();
    let tool0 = tool0_node(&chain);
    let dh = dh();
    let mut rng = Lcg::new(0x5EED_1234);

    let (mut worst_t, mut worst_r) = (0.0f64, 0.0f64);
    for _ in 0..1500 {
        let q = rng.joints();
        let analytic_in_base_link = rz_pi() * forward_kinematics(&dh, &q);
        let (dt, dr) = pose_err(&analytic_in_base_link, &urdf_fk(&chain, &tool0, &q));
        worst_t = worst_t.max(dt);
        worst_r = worst_r.max(dr);
    }
    assert!(
        worst_t < 1e-9 && worst_r < 1e-9,
        "analytic FK disagrees with the URDF: {worst_t} m, {worst_r} rad"
    );
}

/// Documents `state_publisher.rs`'s `joints_for_tf[0] += PI` as the frame hack it is.
///
/// Because joint 1's origin is a pure Z translation, `Rz(pi)` commutes through it,
/// so adding pi to joint 0 makes the published TF tree's `base_link` behave as the
/// UR `base` frame. That is why poses looked up against `base_link` can be handed
/// straight to the controller today - and why `FrameConfig::base_link_is_ur_base`
/// defaults to true. If that offset is ever removed, this test fails and points at
/// the decision instead of at a mirrored robot.
#[test]
fn joint_zero_pi_offset_is_equivalent_to_the_base_rotation() {
    let chain = chain();
    let tool0 = tool0_node(&chain);
    let dh = dh();
    let mut rng = Lcg::new(0xC0FFEE);

    for _ in 0..600 {
        let q = rng.joints();
        let mut shifted = q;
        // FK is 2pi-periodic in joint 0, but `k` range-checks against the URDF
        // limits, so the shifted value has to be wrapped back inside them.
        shifted[0] = wrap_to_pi(q[0] + std::f64::consts::PI);
        let (dt, dr) = pose_err(&forward_kinematics(&dh, &q), &urdf_fk(&chain, &tool0, &shifted));
        assert!(dt < 1e-9 && dr < 1e-9, "offset equivalence broken: {dt} m, {dr} rad");
    }
}

/// `Rz(pi)` is a change of parent frame, not a sign flip on the rotation vector.
#[test]
fn base_frame_conversion_is_not_a_rotation_vector_sign_flip() {
    let dh = dh();
    let q = [0.3, -1.2, 1.1, -0.4, 0.7, 0.2];

    let in_ur_base = forward_kinematics(&dh, &q);
    let in_base_link = rz_pi() * in_ur_base;

    let a = pose_to_array(&in_ur_base);
    let b = pose_to_array(&in_base_link);

    // Translation really does negate x and y.
    assert!((a[0] + b[0]).abs() < 1e-12 && (a[1] + b[1]).abs() < 1e-12);
    assert!((a[2] - b[2]).abs() < 1e-12);

    // The rotation vector does not simply negate. Asserted so the conversion is
    // never "simplified" into a sign flip.
    let naive = [b[0], b[1], b[2], -a[3], -a[4], -a[5]];
    let naive_pose = pose_from_array(naive);
    let (_, dr) = pose_err(&naive_pose, &in_base_link);
    assert!(dr > 1e-3, "the naive sign flip happened to be right; revisit this test");

    // And the conversion is its own inverse.
    let (dt, dr) = pose_err(&(rz_pi() * in_base_link), &in_ur_base);
    assert!(dt < 1e-12 && dr < 1e-12);
}

#[test]
fn pose_strings_round_trip() {
    // The rotation vector's norm must be below pi, or it is not the canonical
    // representative of its rotation and cannot round-trip through a quaternion.
    let p = [0.1, -0.2, 0.3, 0.6, -0.4, 1.1];
    let parsed = parse_pose_string(&pose_to_string(&pose_from_array(p))).unwrap();
    for i in 0..6 {
        assert!((parsed[i] - p[i]).abs() < 1e-12, "component {i}");
    }
    // Tolerates the shapes the driver's own formatters produce.
    assert!(parse_pose_string("p[0,0,0,0,0,0]").is_ok());
    assert!(parse_pose_string("  [ 1.0 , 2.0 ,3.0, 0,0,0 ] ").is_ok());
    assert!(parse_pose_string("p[0,0,0,0,0]").is_err());
    assert!(parse_pose_string("not a pose").is_err());
}

// ---------------------------------------------------------------------------
// Inverse kinematics
// ---------------------------------------------------------------------------

/// Limits straight out of `ur20.urdf`: +-2pi everywhere except the elbow.
fn ur20_limits() -> ([f64; 6], [f64; 6]) {
    let tau = 2.0 * std::f64::consts::PI;
    let pi = std::f64::consts::PI;
    ([-tau, -tau, -pi, -tau, -tau, -tau], [tau, tau, pi, tau, tau, tau])
}

/// Every branch the solver returns must actually reach the pose it was given.
///
/// This is the test that catches a sign error in any of the six angles: a wrong
/// sign still produces a plausible-looking joint vector, but its forward
/// kinematics lands somewhere else.
#[test]
fn every_ik_branch_reaches_the_requested_pose() {
    let dh = dh();
    let mut rng = Lcg::new(0xABCD_0001);

    let (mut checked, mut worst_t, mut worst_r) = (0usize, 0.0f64, 0.0f64);
    for _ in 0..600 {
        let q = rng.joints();
        let target = forward_kinematics(&dh, &q);
        let sols = inverse_kinematics(&dh, &target);
        assert!(!sols.is_empty(), "a pose generated by FK must have an IK solution");
        assert!(sols.len() <= 8, "at most eight branches, got {}", sols.len());

        for s in &sols {
            assert!(s.iter().all(|v| v.is_finite()), "non-finite joint value: {s:?}");
            let (dt, dr) = pose_err(&forward_kinematics(&dh, s), &target);
            worst_t = worst_t.max(dt);
            worst_r = worst_r.max(dr);
            checked += 1;
        }
    }
    assert!(checked > 4000, "expected many branches, only checked {checked}");
    assert!(
        worst_t < 1e-9 && worst_r < 1e-9,
        "an IK branch does not reach its pose: {worst_t} m, {worst_r} rad"
    );
}

/// The configuration the pose came from must be among the branches returned.
#[test]
fn ik_recovers_the_generating_configuration() {
    let dh = dh();
    let mut rng = Lcg::new(0xABCD_0002);

    for _ in 0..400 {
        let q = rng.joints();
        let wrapped: Vec<f64> = q.iter().map(|v| wrap_to_pi(*v)).collect();
        let sols = inverse_kinematics(&dh, &forward_kinematics(&dh, &q));

        let found = sols.iter().any(|s| {
            (0..6).all(|i| {
                let d = wrap_to_pi(s[i] - wrapped[i]).abs();
                d < 1e-7
            })
        });
        assert!(found, "IK lost the generating configuration {q:?}");
    }
}

/// Singular configurations must degrade, not explode.
#[test]
fn singular_configurations_stay_finite() {
    let dh = dh();
    let cases: [[f64; 6]; 5] = [
        // Wrist singularity: theta5 == 0 makes the theta6 split free.
        [0.0, -1.2, 1.0, -0.3, 0.0, 0.4],
        [0.7, -0.9, 1.4, 0.2, 0.0, -1.1],
        // Elbow fully extended.
        [0.3, -1.0, 0.0, -0.5, 0.8, 0.1],
        // Shoulder: wrist centre near the z0 axis.
        [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0],
        // All zeros, the most degenerate pose the arm has.
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ];

    for q in cases {
        let target = forward_kinematics(&dh, &q);
        let sols = inverse_kinematics(&dh, &target);
        for s in &sols {
            assert!(s.iter().all(|v| v.is_finite()), "non-finite at {q:?}: {s:?}");
            let (dt, dr) = pose_err(&forward_kinematics(&dh, s), &target);
            assert!(dt < 1e-7 && dr < 1e-7, "singular branch off by {dt} m, {dr} rad at {q:?}");
        }
    }
}

/// An unreachable pose yields nothing rather than nonsense.
#[test]
fn unreachable_poses_return_no_solutions() {
    let dh = dh();
    // Far outside the UR20's ~1.75 m reach.
    let far = pose_from_array([5.0, 0.0, 0.5, 0.0, 0.0, 0.0]);
    assert!(inverse_kinematics(&dh, &far).is_empty());
}

/// Turn expansion must stay inside the limits and must not lose the base branch.
#[test]
fn filtered_solutions_respect_joint_limits() {
    let dh = dh();
    let (lower, upper) = ur20_limits();
    let mut rng = Lcg::new(0xABCD_0003);

    for _ in 0..150 {
        let q = rng.joints();
        let target = forward_kinematics(&dh, &q);
        let sols = inverse_kinematics_filtered(&dh, &target, &lower, &upper);
        assert!(!sols.is_empty());
        for s in &sols {
            for i in 0..6 {
                assert!(
                    s[i] >= lower[i] && s[i] <= upper[i],
                    "joint {i} = {} outside [{}, {}]",
                    s[i], lower[i], upper[i]
                );
            }
            let (dt, dr) = pose_err(&forward_kinematics(&dh, s), &target);
            assert!(dt < 1e-9 && dr < 1e-9);
        }
    }
}

/// Branch selection returns the weighted-nearest configuration, not just any.
#[test]
fn select_nearest_minimises_weighted_distance() {
    let dh = dh();
    let (lower, upper) = ur20_limits();
    let mut rng = Lcg::new(0xABCD_0004);

    for _ in 0..150 {
        let q = rng.joints();
        let target = forward_kinematics(&dh, &q);
        let sols = inverse_kinematics_filtered(&dh, &target, &lower, &upper);
        let picked = select_nearest(&sols, &q).expect("a solution exists");

        let best = sols
            .iter()
            .map(|s| configuration_distance(s, &q))
            .fold(f64::INFINITY, f64::min);
        assert!(
            (configuration_distance(&picked, &q) - best).abs() < 1e-12,
            "select_nearest did not pick the minimum"
        );
        // Seeding with the generating configuration should land back on it.
        let (dt, dr) = pose_err(&forward_kinematics(&dh, &picked), &target);
        assert!(dt < 1e-9 && dr < 1e-9);
    }
}

/// The analytic Jacobian must match a central finite difference of FK.
#[test]
fn jacobian_matches_finite_differences() {
    let dh = dh();
    let mut rng = Lcg::new(0xABCD_0005);
    let h = 1e-6;

    let mut worst = 0.0f64;
    for _ in 0..120 {
        let q = rng.joints();
        let j = jacobian(&dh, &q);

        for i in 0..6 {
            let (mut qp, mut qm) = (q, q);
            qp[i] += h;
            qm[i] -= h;
            let fp = forward_kinematics(&dh, &qp);
            let fm = forward_kinematics(&dh, &qm);

            let lin = (fp.translation.vector - fm.translation.vector) / (2.0 * h);
            let ang = (fp.rotation * fm.rotation.inverse()).scaled_axis() / (2.0 * h);

            for r in 0..3 {
                worst = worst.max((j[(r, i)] - lin[r]).abs());
                worst = worst.max((j[(r + 3, i)] - ang[r]).abs());
            }
        }
    }
    assert!(worst < 1e-6, "Jacobian disagrees with finite differences by {worst}");
}

// Iteration counts above are deliberately modest: this suite runs in a debug
// build, where nalgebra and the URDF chain traversal are slow enough that a few
// thousand samples per test turns `cargo test` into a multi-minute wait. Raise
// them locally with --release when changing the kinematics.
