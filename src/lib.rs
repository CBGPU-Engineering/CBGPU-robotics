//! Local device SDK. Backend-issued plans are data, never executable code.
pub mod adapter;
pub mod client;
pub mod controller;
pub mod protocol;
pub mod remote;
pub mod remote_protocol;
pub mod ros2_state;
pub mod simulator;
pub type Result<T> = anyhow::Result<T>;
pub fn fail(message: impl Into<String>, _status: u16) -> anyhow::Error {
    anyhow::anyhow!(message.into())
}
pub fn require(ok: bool, message: &str) -> Result<()> {
    anyhow::ensure!(ok, "{message}");
    Ok(())
}
pub fn valid_id(value: &str) -> bool {
    uuid::Uuid::parse_str(value).is_ok_and(|id| id.to_string() == value)
}
pub fn now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}
pub fn private_file(path: &std::path::Path) -> Result<()> {
    let m = std::fs::symlink_metadata(path)?;
    require(
        m.file_type().is_file(),
        "Credential/ledger path must be a regular file",
    )?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        require(
            m.permissions().mode() & 0o077 == 0,
            "Private file permissions must be 600",
        )?;
    }
    Ok(())
}
#[cfg(any(feature = "ros2-jazzy", feature = "ros2-humble"))]
pub mod ros2;
