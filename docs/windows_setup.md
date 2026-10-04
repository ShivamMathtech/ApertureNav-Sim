# Windows 11 → WSL2 → ROS 2 Jazzy / Gazebo Harmonic

The native Windows dashboard does not need ROS. The ROS simulator runs inside
Ubuntu 24.04. Official references:

- https://learn.microsoft.com/windows/wsl/install
- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- https://gazebosim.org/docs/harmonic/ros_installation/
- https://gazebosim.org/docs/harmonic/ros2_integration/

1. Enable CPU virtualization in firmware (Intel VT-x / AMD-V).
2. Open an Administrator PowerShell and run:

```powershell
wsl --install -d Ubuntu-24.04
wsl --update
wsl --set-default-version 2
```

Restart when Windows requests it. Open Ubuntu, create your Linux username and
password, then run the remaining Linux commands inside Ubuntu.

3. Update Ubuntu and install locale / repository prerequisites:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

4. Install the official ROS apt-source package, then ROS desktop. This follows
the ROS apt-source release mechanism. If the official procedure changes, use
the installation page linked above.

```bash
ROS_APT_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import sys,json; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
source /opt/ros/jazzy/setup.bash
```

5. Install the matched Gazebo / ROS bridge packages:

```bash
sudo apt install -y ros-jazzy-ros-gz
```

6. Test ROS messaging. In terminal A:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

In terminal B:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

7. Test graphics with `gz sim shapes.sdf` and `rviz2` (each in its own terminal).
WSLg supplies GUI support on Windows 11. Keep graphics drivers and WSL updated.

8. Extract the project under your Linux home, for example `~/aperturenav_sim`.
Using `/mnt/c` for colcon/node_modules can substantially slow builds. Then:

```bash
cd ~/aperturenav_sim
bash scripts/setup_ubuntu.sh
bash scripts/build.sh
bash scripts/run_sim.sh
```

## Beginner glossary

- **WSL2** runs a Linux environment on Windows.
- **ROS** is the robotics communication and software framework.
- A **workspace** is a folder containing ROS packages and build outputs.
- **colcon** builds and tests those packages.
- A **node** is an independent ROS program.
- A **topic** carries typed messages between nodes.
- **TF** tracks coordinate frame relationships at timestamps.
- **Gazebo** simulates the world and sensors.
- **RViz** visualizes ROS data; it is not a physics simulator.

No Gazebo/PX4 hardware link is created by these instructions.
