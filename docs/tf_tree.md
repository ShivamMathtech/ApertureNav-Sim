# Coordinate frames

```mermaid
flowchart TD
 world --> map
 map --> odom
 odom --> base_link
 base_link --> camera_link
 camera_link --> camera_optical_frame
 camera_link --> depth_camera_link
 camera_optical_frame --> depth_camera_optical_frame
 base_link --> imu_link
 world --> aperture_frame
```

`world → map → odom` are identity static transforms. Odometry updates
`odom → base_link`; robot_state_publisher publishes the fixed camera/IMU/rotor
transforms. Localization publishes `world → aperture_frame` at the image stamp.
RGB and depth share an optical center in this baseline. No LiDAR link is
advertised because LiDAR is not included.
