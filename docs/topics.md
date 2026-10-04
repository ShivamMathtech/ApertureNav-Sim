# ROS topics and services

| Topic | Type | Direction / purpose |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Gazebo → ROS simulation clock |
| `/drone/camera/image_raw` | sensor_msgs/Image | RGB camera → detector |
| `/drone/camera/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |
| `/drone/depth/image_raw` | sensor_msgs/Image | Depth in metres (or 16UC1 millimetres) |
| `/drone/depth/points` | sensor_msgs/PointCloud2 | Gazebo RGB-D cloud |
| `/drone/imu` | sensor_msgs/Imu | Gazebo IMU |
| `/drone/odom` | nav_msgs/Odometry | Gazebo body pose / velocity |
| `/aperture/detection` | drone_interfaces/ApertureDetection | Four ordered image corners and candidate score |
| `/aperture/pose` | drone_interfaces/AperturePose | World pose, dimensions and plane residual |
| `/aperture/debug_image` | sensor_msgs/Image | Detector overlay |
| `/drone/camera/compressed` | sensor_msgs/CompressedImage | JPEG overlay for web API |
| `/aperture/marker` | visualization_msgs/Marker | Opening outline |
| `/planner/trajectory` | nav_msgs/Path | Pre-entry, center and exit points |
| `/aperture/safe_to_enter` | std_msgs/Bool | Clearance status |
| `/aperture/minimum_clearance` | std_msgs/Float32 | Remaining clearance after reserves |
| `/drone/cmd_vel` | geometry_msgs/Twist | Body-frame velocity command → Gazebo |
| `/mission/state` | std_msgs/String | Mission state |
| `/sim/telemetry` | std_msgs/String | JSON aggregate for web client |
| `/ui/command` | std_msgs/String | start, pause, resume, reset, abort, hover, estop |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | Node diagnostics |

`/start_mission`, `/abort_mission`, `/reset_simulation`: std_srvs/Trigger.
`/world/aperture/control`: ros_gz_interfaces/ControlWorld bridged to Gazebo.
A reset response means the request was accepted; diagnostics report completion
or failure of the asynchronous world-control operation.

Sensor subscriptions use sensor-data QoS. Localization rejects RGB/depth pairs
more than 100 ms apart. Command names and frame conventions are documented in
source; no unimplemented ROS actions are advertised.
