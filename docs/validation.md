# Validation report — 2026-10-04

## Passed in the delivery environment

- 13 automated tests: default/rotated traversal, unsafe-opening abort, exact
  threshold rejection, swept collision, PID limits, minimum-jerk endpoints,
  sensor timeout, invalid transitions, reset determinism, depth localization,
  synthetic vision under blur/noise/dim lighting, multiple candidates, REST
  validation/commands and WebSocket telemetry.
- Six-case offline width benchmark: 1.20, 1.00, 0.80 and 0.60 m traversed;
  0.50 and 0.45 m rejected. Zero simulated collisions.
- React/TypeScript type checking and Vite production build.
- Python compilation, Bash syntax and XML parsing for generated assets.
- Source ZIP integrity check.

`test-results.txt` records the actual unittest output. Sample benchmark outputs
are included under `experiments/results/`. Results are from the explicitly
labelled idealized simulator; they do not represent Gazebo or camera accuracy.

## Not executed

- `colcon build`, `colcon test` and Gazebo/RViz acceptance: ROS 2, colcon and
  Gazebo are not installed in the delivery environment.
- Browser visual/end-to-end automation: the Chromium download was unavailable.
  The frontend was type-checked and built; the backend was tested directly.
- Native Windows / WSL installation and GPU rendering.
- Real drone, PX4, aerodynamic dynamics and advanced planners.

The build emits a bundle-size warning for Three.js; it is not a build failure.
The test environment emits an upstream anyio deprecation warning; all tests pass.
