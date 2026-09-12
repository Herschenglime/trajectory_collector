# Agent Guidelines for `husky_ws`

## ROS 2 Log Inspection
* **Inspect logs directly from source**: Always read ROS 2 logs directly from `~/.ros/log` (or `~/.ros/log/latest/launch.log` / `~/.ros/log/<timestamp>/launch.log`). Do NOT ask the user to redirect `ROS_LOG_DIR` into the workspace or pipe output with `tee`.
* **Sandbox Permissions**: If sandbox read access has not yet been granted to `~/.ros/log`, request read access for the entire `~/.ros/log` directory (e.g., via `list_dir` on `/home/pgrau/.ros/log`) rather than attempting individual files or running unsandboxed shell commands.
