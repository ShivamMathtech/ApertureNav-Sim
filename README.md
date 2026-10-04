# ApertureNav-Sim

**A runnable research baseline for rectangular-aperture navigation**, with a
local simulation dashboard, shared navigation algorithms and a ROS 2 Jazzy /
Gazebo Harmonic integration source tree. Created for Shivam Singh / MathTech.

> Release 0.1 is a baseline, not completion of every item in the attached master
> specification. Offline algorithms, perception fixtures, API and frontend can
> be tested independently. The ROS/Gazebo integration has not been built or run
> in the delivery environment. Review [implementation status](docs/status.md)
> before using its results as robotics evidence.

## Fastest start: Windows or Linux dashboard

Use **Python 3.12**. The ZIP includes the compiled dashboard, so Node.js is only
needed if you edit the frontend.

Windows PowerShell, from this extracted directory:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m uvicorn dashboard.backend.app:app --host 127.0.0.1 --port 8000
```

If PowerShell blocks activation, use the environment interpreter directly:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m uvicorn dashboard.backend.app:app --host 127.0.0.1 --port 8000
```

Ubuntu / WSL:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
bash scripts/run_dashboard.sh
```

Open **http://127.0.0.1:8000**. Start a mission, inspect the 3D view, pause,
reset, change the opening dimensions and download experiment data.
The local dashboard uses an explicitly labelled **kinematic simulator** with
known aperture geometry. Its FPV mode is a synthetic 3D viewpoint.

## Offline experiments and tests

```bash
python -m aperturenav.cli
python -m aperturenav.cli --benchmark
python -m pip install -r requirements-dev.txt
python -m unittest discover -s tests -v
```

The default mission traverses a 0.80 × 0.60 m opening. The width benchmark also
includes deliberately unsafe 0.50 and 0.45 m openings, which should abort.
Do not interpret a benchmark containing deliberate failure cases as a learned
perception success rate. CSV and JSON outputs are in `experiments/results/`.

## ROS 2 + Gazebo

Follow [Windows/WSL setup](docs/windows_setup.md), then:

```bash
bash scripts/install_ros_dependencies.sh
bash scripts/build.sh
bash scripts/run_sim.sh
```

In a second WSL terminal, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source drone_ws/install/setup.bash
ros2 service call /start_mission std_srvs/srv/Trigger '{}'
```

This launch generates a world from the same YAML used by navigation, opens
Gazebo and RViz, bridges RGB-D/IMU/odometry and starts perception and navigation.
The Gazebo drone uses an idealized velocity controller with gravity disabled;
it does not simulate motor thrust, aerodynamic effects or PX4.

For algorithm debugging only, `bash scripts/run_sim.sh ground_truth_aperture:=true`
uses configured aperture geometry and labels telemetry accordingly. Default is
`false`; then a frame must be localized from RGB-D data.

To view ROS telemetry in the web dashboard, use a separate environment to keep
ROS's NumPy/OpenCV ABI intact:

```bash
source /opt/ros/jazzy/setup.bash
source drone_ws/install/setup.bash
python3 -m venv --system-site-packages .venv-ros
source .venv-ros/bin/activate
python -m pip install fastapi uvicorn
APERTURE_MODE=ros bash scripts/run_dashboard.sh
```

Ctrl+C stops the foreground process. Stop each terminal you started. `/abort_mission`
latches an abort; `/reset_simulation` calls the Gazebo world reset service and
clears local mission/controller state after success. Test these in your ROS
installation before relying on the integrated workflow.

## Edit the dashboard

Use Node.js 22.12 or newer:

```bash
cd dashboard/frontend
npm ci
npm run dev
# Or regenerate the included production files:
npm run build
```

Vite proxies `/api` and `/ws` to the backend at port 8000. Frontend is React +
TypeScript + Three.js. API documentation: http://127.0.0.1:8000/docs.

## Project map

- `aperturenav/`: canonical pure Python navigation, geometry, perception and simulator.
- `drone_ws/src/`: ROS interfaces, adapters, description, worlds and bringup packages.
- `dashboard/`: FastAPI service and React/TypeScript/Three.js frontend.
- `config/default.yaml`: geometry, safety, control, mission and experiment settings.
- `tests/`: algorithm, synthetic perception and API tests.
- `experiments/`: recorded CSV/JSON experiments.
- `scripts/`: setup, build, launch, test and packaging helpers.
- `docs/`: architecture, topic/TF maps, setup, status, validation and troubleshooting.

`scripts/build.sh` refreshes the ROS package's copy of the canonical Python core.
Run `python scripts/sync_ros_core.py` after editing it if invoking colcon manually.

## GitHub

A GitHub Actions workflow runs Python tests and the frontend build. No repository
was selected for this delivery, so this ZIP has not been pushed to GitHub.
To publish yourself to a new **empty** repository:

```bash
git init -b main
git add .
git commit -m "Add ApertureNav-Sim research baseline"
git remote add origin https://github.com/ShivamMathtech/YOUR_REPOSITORY.git
git push -u origin main
```

Use a separate development branch if adding this to an existing repository;
do not overwrite existing files without reviewing the diff. License: MIT.
