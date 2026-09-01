//! Saving and loading planned trajectories as JSON files.
//!
//! One file per trajectory under the trajectory directory (`UR_TRAJECTORY_DIR`,
//! default `trajectories/`). Files rather than Redis keys because a trajectory is
//! a few hundred waypoints of numbers that wants to be diffed, reviewed and
//! version-controlled, and because a Redis `SPValue` array of that size is
//! expensive to encode on every request.
//!
//! The stored request is kept alongside the result so a trajectory can be
//! re-planned when the limits change, rather than only replayed.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use super::plan::{PlanRequest, PlannedTrajectory};

/// Bumped whenever the stored shape changes incompatibly.
pub const SCHEMA_VERSION: u32 = 1;

/// Where trajectories live when `UR_TRAJECTORY_DIR` is not set.
pub const DEFAULT_TRAJECTORY_DIR: &str = "trajectories";

/// One trajectory on disk.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StoredTrajectory {
    pub schema_version: u32,
    pub name: String,
    /// Seconds since the Unix epoch. Deliberately not a formatted timestamp: this
    /// crate has no date-time dependency and does not need one.
    pub created_unix_seconds: u64,
    /// The problem this came from, so it can be re-planned rather than only replayed.
    pub request: Option<PlanRequest>,
    pub trajectory: PlannedTrajectory,
}

/// The directory trajectories are read from and written to.
pub fn trajectory_dir() -> PathBuf {
    std::env::var("UR_TRAJECTORY_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(DEFAULT_TRAJECTORY_DIR))
}

/// Reject anything that is not a plain file stem.
///
/// A trajectory name arrives from Redis and is interpolated into a filesystem
/// path, so it gets the same treatment `command_server` gives `command_type`
/// before that becomes a template filename: an arbitrary string must not be able
/// to reach outside the directory it is supposed to name.
pub fn validate_name(name: &str) -> Result<(), String> {
    if name.is_empty() {
        return Err("a trajectory name cannot be empty".to_string());
    }
    if name.len() > 128 {
        return Err(format!("trajectory name '{name}' is too long"));
    }
    if !name
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-' || c == '.')
        || name.contains("..")
        || name.starts_with('.')
    {
        return Err(format!(
            "trajectory name '{name}' may only contain letters, digits, '_', '-' and \
             '.', and may not start with '.' or contain '..'"
        ));
    }
    Ok(())
}

/// The path a named trajectory occupies.
pub fn path_for(name: &str) -> Result<PathBuf, String> {
    validate_name(name)?;
    Ok(trajectory_dir().join(format!("{name}.json")))
}

/// Write a trajectory, creating the directory if it does not exist.
pub fn save(
    name: &str,
    trajectory: &PlannedTrajectory,
    request: Option<&PlanRequest>,
) -> Result<PathBuf, String> {
    let path = path_for(name)?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|e| format!("could not create {}: {e}", parent.display()))?;
    }

    let stored = StoredTrajectory {
        schema_version: SCHEMA_VERSION,
        name: name.to_string(),
        created_unix_seconds: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
        request: request.cloned(),
        trajectory: trajectory.clone(),
    };

    let json = serde_json::to_string_pretty(&stored)
        .map_err(|e| format!("could not serialise trajectory '{name}': {e}"))?;
    std::fs::write(&path, json).map_err(|e| format!("could not write {}: {e}", path.display()))?;
    Ok(path)
}

/// Read a trajectory by name.
pub fn load(name: &str) -> Result<StoredTrajectory, String> {
    let path = path_for(name)?;
    load_path(&path)
}

/// Read a trajectory from an explicit path.
pub fn load_path(path: &Path) -> Result<StoredTrajectory, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("could not read {}: {e}", path.display()))?;
    let stored: StoredTrajectory = serde_json::from_str(&text)
        .map_err(|e| format!("{} is not a valid trajectory: {e}", path.display()))?;

    if stored.schema_version != SCHEMA_VERSION {
        return Err(format!(
            "{} was written by schema version {} but this build expects {}; re-plan it",
            path.display(),
            stored.schema_version,
            SCHEMA_VERSION
        ));
    }
    Ok(stored)
}

/// Every trajectory name in the directory, sorted. An absent directory is empty.
pub fn list() -> Result<Vec<String>, String> {
    let dir = trajectory_dir();
    if !dir.exists() {
        return Ok(Vec::new());
    }
    let mut names = Vec::new();
    for entry in
        std::fs::read_dir(&dir).map_err(|e| format!("could not read {}: {e}", dir.display()))?
    {
        let entry = entry.map_err(|e| format!("could not read an entry in {}: {e}", dir.display()))?;
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) == Some("json") {
            if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                names.push(stem.to_string());
            }
        }
    }
    names.sort();
    Ok(names)
}
