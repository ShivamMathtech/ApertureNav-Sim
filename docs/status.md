# Implementation status — 0.1.0

This release provides a runnable baseline, rather than claiming the 64-section
master specification is complete. The original requirements are retained in
`docs/original-requirements.txt`.

| Area | Delivered | Limits / verification |
|---|---|---|
| Local simulator | Seeded kinematics, state machine, PID, swept collision checks | Uses known aperture geometry; no physical flight dynamics |
| Mission states | All 16 named states and guarded transitions | Integrated with controller in one ROS navigator process |
| Clearance | Rectangular envelope, relative yaw, bounded yaw uncertainty, offsets and margin | Vertical walls only; no roll/pitch footprint model |
| Planning | Three aligned waypoints and minimum-jerk interpolation function | Mission uses waypoint PID; MPC, RRT*, A*, minimum snap not implemented |
| Perception | OpenCV rectangular candidate detector, rim-depth plane fit | Synthetic tests only; no trained segmentation or calibrated confidence |
| Localization | Camera intrinsics, depth, timestamped TF transform, plane normal | Conservative rejection of nonvertical/degenerate planes; covariance not implemented |
| Safety | Stale odometry/depth, lost target, invalid depth, small opening, collision abort | Forward depth ROI and rectangular map model; no full obstacle avoidance |
| ROS packages | Core, adapters, custom interfaces, detector, localizer, mission, description, Gazebo, bringup | Source compilation and ROS integration unverified here |
| Gazebo | Generated rectangular worlds, RGB-D, point cloud, IMU, odometry, velocity control | Idealized body motion, no rotor dynamics; physics acceptance unverified |
| Worlds | Empty, standard, small, rotated, low light | Circle, irregular, occluded and multiple-opening autonomous traversal not implemented |
| Dashboard | 3D views, live telemetry, controls, scenarios, charts, result downloads | Local mode executable; ROS mode needs ROS environment; RGB JPEG API provided |
| Logging | Config, telemetry CSV, events CSV, summary JSON | Not all research metrics; no fabricated FPS/latency or perception accuracy |
| Reset | Local reset and asynchronous Gazebo world reset integration | Gazebo reset must be acceptance-tested on Ubuntu |
| ROS architecture | Separate perception/localization nodes; navigation/controller/safety/logging in one process | Does not reproduce all 14 requested independent packages, services/actions or custom messages |
| PX4 / advanced planners | Not included | Optional subsequent engineering phase |
| Production readiness | Not claimed | Requires ROS builds, integrated acceptance, coverage expansion and dynamics validation |

The supplied source can be developed further without PX4. Passing local tests
proves the tested numerical/API behavior only; it does not validate Gazebo or a
real aircraft. The baseline is intended for simulation education and research.
