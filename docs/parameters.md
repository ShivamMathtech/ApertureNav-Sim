# Parameters

All numerical core configuration is in `config/default.yaml`. ROS-specific
parameters include `config_file`, `use_sim_time`, `ground_truth_aperture`,
`results_directory`, detector `min_area` and detector `rate`.

| YAML group | Parameters | Units / meaning |
|---|---|---|
| root | seed, dt, max_duration | RNG seed, integration seconds, offline limit |
| drone | width, length, height, spawn | Outer collision envelope and XYZ position, metres |
| aperture | center, width, height, yaw, thickness | Metres, yaw radians |
| safety | margin, position_uncertainty, yaw_uncertainty | Per-side metres and angular bound radians |
| safety | stale_sensor_timeout, target_lost_timeout | Seconds |
| controller | kp, ki, kd, integral_limit | PID gains and integral clamp |
| controller | max_speed, entry_speed, exit_speed | Metres/second |
| controller | max_acceleration, max_yaw_rate | m/s² and rad/s |
| mission | approach_distance, exit_distance | Metres along aperture normal |
| mission | position_tolerance, yaw_tolerance, timeout | Metres, radians, seconds |
| environment | wind_std | Local simulator velocity disturbance std, m/s |

Clearance is the remaining reserve after subtracting the projected vehicle
footprint, lateral/vertical error, margin and position uncertainty. Yaw
uncertainty is maximized over its full interval. Clearance ≤ 1e-9 is rejected.
The offline wind parameter is not a Gazebo aerodynamic wind model.

Camera pose, FOV, rates and world lighting currently live in the world generator;
they are explicit code parameters, not fully exposed through YAML. These are
listed as remaining configuration work in the requirements status.
