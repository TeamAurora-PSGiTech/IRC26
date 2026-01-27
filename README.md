# IRC26 - ROS 2 Workspace

A ROS 2 Humble workspace for robotics control and simulation, featuring packages for arm control, motor control, autonomous systems, and rover/arm simulation.

## Overview

This workspace contains the following ROS 2 packages:

- **arm_ctrl** - Arm control package
- **arm_sim** - Arm simulation package
- **autonomous** - Autonomous navigation and control systems
- **motor_ctrl** - Motor control interface
- **rover_sim** - Rover simulation package

## Prerequisites

- ROS 2 Humble
- Python 3.10
- colcon build tool
- pytest (for testing)

## Installation

1. Clone this repository:
```bash
git clone <repository-url> ~/IRC26
cd ~/IRC26
```

2. Install system/ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. (Optional) Install Python dependencies used by the packages and tests:

```bash
python -m pip install -r requirements.txt
```

4. Build the workspace:

```bash
colcon build
```

## Usage

### Setup Environment

Source the workspace setup file:

**Bash:**
```bash
source install/setup.bash
```

**Zsh:**
```bash
source install/setup.zsh
```

**PowerShell:**
```powershell
. install/setup.ps1
```

### Running Packages

After sourcing the workspace, you can run ROS 2 nodes from the packages:

```bash
ros2 run <package_name> <node_name>
```

To run the wheel interface for the `motor_ctrl` package (wheel control node), use:

```bash
ros2 run motor_ctrl interface
```

### Useful Aliases

Add these to your shell rc file (e.g., `~/.bashrc` or `~/.zshrc`) for quick access:

```bash
alias tmux8="$HOME/tmux-start.sh"
alias motor="ros2 run motor_ctrl interface"
alias nav="ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$HOME/IRC26/src/config/nav2_params.yaml"
alias ekf="ros2 run robot_localization ekf_node --ros-args --params-file $HOME/IRC26/src/config/ekf.yaml"
alias rtab="ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/camera/color/camera_info frame_id:=base_link odom_frame_id:=odom publish_tf:=true visual_odometry:=true approx_sync:=true rtabmap_viz:=false delete_db_on_start:=true"
alias tf2="ros2 run tf2_ros static_transform_publisher 0.15 0.0 0.25 0 0 0 base_link camera_link"
alias rscam="ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=1"
```

### Nav2 / RTAB-Map quick launch (multi-terminal)

Use the helper script to start the Nav2 stack in separate terminals:

```bash
chmod +x scripts/run_nav2_stack.sh
./scripts/run_nav2_stack.sh
```

Notes:
- The script opens multiple GUI terminals. On Jetson without an active display session, it will exit with an error.
- You can tweak behavior via environment variables: `USE_SIM_TIME`, `INTERFACE_MODE` (`auto` or `const`), `SEND_TEST_GOAL`, `GOAL_X`, `GOAL_Y`, `SETUP_SCRIPT`.


If you see a "Permission denied" error when opening a serial device (for example `/dev/ttyUSB0` or `/dev/ttyACM0`), prefer a persistent and secure fix using a udev rule and the dialout group instead of `chmod 666`.

Quick temporary fix (not recommended for long-term use):

```bash
sudo chmod 666 "/dev/ttyUSB0"
# or
sudo chmod 666 "/dev/ttyACM0"
```

Recommended persistent fix (udev + dialout):

1. Create a file `/etc/udev/rules.d/99-irc26-serial.rules` with the contents shown in `udev/99-irc26-serial.rules` in this repository.
2. Reload udev rules and trigger them:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

3. Add your user to the `dialout` group so you can access serial devices without sudo:

```bash
sudo usermod -a -G dialout $USER
# then either log out and log back in, or run
newgrp dialout
```

This gives controlled access to serial devices and avoids world-writable device files.

#### Getting Sensor Data

1. Build the `sensors` package (or the whole workspace if you prefer):

```bash
colcon build --packages-select sensors
```

2. Source the workspace setup file (see options above for your shell), for example:

```bash
source install/setup.zsh
```

3. Launch the GPS interface node:

```bash
ros2 run sensors gps_interface
```

### Testing

Run tests for all packages:
```bash
colcon test
```

View test results:
```bash
colcon test-result --verbose
```

## Workspace Structure

```
IRC26/
├── src/              # Source code for all packages
│   ├── arm_ctrl/
│   ├── arm_sim/
│   ├── autonomous/
│   ├── motor_ctrl/
│   └── rover_sim/
├── build/            # Build artifacts (generated)
├── install/          # Installation files (generated)
├── log/              # Build and test logs (generated)
└── README.md
```

## Package Information

All packages are ament_python packages with the following structure:
- Version: 0.0.0
- Maintainer: sandy <sandytheadventurer@gmail.com>
- License: TODO: License declaration

### Development

Each package includes test dependencies:
- ament_copyright
- ament_flake8
- ament_pep257
- python3-pytest




## Building Individual Packages

To build a specific package:
```bash
colcon build --packages-select <package_name>
```

## Cleaning the Workspace

To clean build artifacts:
```bash
rm -rf build install log
```

How to Navigate to GPS Coordinates
Prerequisites - Run these in separate terminals:
Terminal 1: GPS Interface

"""
cd ~/IRC26
source install/setup.bash
ros2 run sensors gps_interface
"""

this will run the gps node 
"""
Terminal 2: NavSat Transform (converts GPS to map coordinates)

cd ~/IRC26
source install/setup.bash
ros2 launch sensors navsat_transform.launch.py
"""

"""
Terminal 3: Nav2 Stack (needs RTAB-Map running)

cd ~/IRC26
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$HOME/IRC26/src/config/nav2_params.yaml
"""
Send the Rover to GPS Coordinates

cd ~/IRC26
source install/setup.bash
ros2 run sensors gps_waypoint --ros-args -p target_lat:=<LATITUDE> -p target_lon:=<LONGITUDE>

IMPORTANT : I HAVE SET THE GPS DATUM AS THE OUR HOTEL (WE NEED TO CHANGE ACCORDINGLY)

## Contributing

TODO: Add contribution guidelines

## License

TODO: License declaration

## Contact

Maintainer: sandy  
Email: [sandytheadventurer@gmail.com](mailto:sandytheadventurer@gmail.com)
