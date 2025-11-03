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

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
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
- Maintainer: sandy (sandytheadventurer@gmail.com)
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
Email: sandytheadventurer@gmail.com