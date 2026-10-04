# Troubleshooting

| Symptom | Action |
|---|---|
| `ros2: command not found` | Use Ubuntu terminal and `source /opt/ros/jazzy/setup.bash`. |
| Package not found | Run build; source `drone_ws/install/setup.bash` in each new terminal. |
| colcon failure | Read the first compiler/import error. Run `rosdep install --from-paths src --ignore-src -r -y`; rebuild affected package. |
| GUI / RViz missing | Run `wsl --update` in Windows, update GPU drivers, restart WSL. Verify WSLg with an unrelated GUI app. |
| Gazebo slow / OpenGL error | Try `LIBGL_ALWAYS_SOFTWARE=1` as a diagnostic; it is slower. Reduce image resolution/rates in `aperturenav/world.py`. |
| No bridge topics | Inspect `gz topic -l`, `ros2 topic list` and `drone_gazebo/config/bridge.yaml`; sensor topic naming can vary with Gazebo versions. |
| No image | `ros2 topic hz /drone/camera/image_raw`; ensure physics is running and the sensor plugin loads. |
| Missing TF | `ros2 run tf2_ros tf2_echo world camera_optical_frame`; check odometry and robot_state_publisher. |
| Waiting in SEARCH | Inspect `/aperture/debug_image`, finite rim depth, frame contrast, intrinsics and TF. No validated candidate means no autonomous target. |
| EMERGENCY_HOVER | Inspect event reason / diagnostics: stale sensor, invalid depth, lost target, insufficient clearance or obstacle. Reset after resolving. |
| Python ABI/import problem in ROS | Use Ubuntu Python 3.12 and system ROS NumPy/OpenCV. Avoid installing a second NumPy/OpenCV in the ROS environment. |
| Dashboard disconnected | Start backend on port 8000; inspect its terminal. In ROS mode, check `/sim/telemetry` exists. |
| Frontend Node error | Use Node 22.12+; `npm ci`; `npm run build`. Do not install Linux-only Rollup packages manually on Windows. |
| PowerShell activation blocked | Invoke `.venv\Scripts\python.exe` directly, as shown in README. |
| Port 8000 in use | Stop the previous server or change the port and Vite proxy together. |
| Reset rejected | Verify `/world/aperture/control` service bridge and inspect diagnostics. Local mode reset needs no ROS. |

The simulator does not silently substitute ground truth when perception fails.
The explicit ground-truth launch option is for geometry/control debugging only.
