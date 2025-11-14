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

1. Install system/ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

1. (Optional) Install Python dependencies used by the packages and tests:

```bash
python -m pip install -r requirements.txt
```

1. Build the workspace:

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

If you see a "Permission denied" error when opening a serial device (for example `/dev/ttyUSB0` or `/dev/ttyACM0`), prefer a persistent and secure fix using a udev rule and the dialout group instead of `chmod 666`.

Quick temporary fix (not recommended for long-term use):

```bash
sudo chmod 666 "/dev/ttyUSB0"
# or
sudo chmod 666 "/dev/ttyACM0"
```

Recommended persistent fix (udev + dialout):

1. Create a file `/etc/udev/rules.d/99-irc26-serial.rules` with the contents shown in `udev/99-irc26-serial.rules` in this repository.
1. Reload udev rules and trigger them:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

1. Add your user to the `dialout` group so you can access serial devices without sudo:

```bash
sudo usermod -a -G dialout $USER
# then either log out and log back in, or run
newgrp dialout
```

This gives controlled access to serial devices and avoids world-writable device files.

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

## Contributing

TODO: Add contribution guidelines

## License

TODO: License declaration

## Contact

Maintainer: sandy  
Email: [sandytheadventurer@gmail.com](mailto:sandytheadventurer@gmail.com)