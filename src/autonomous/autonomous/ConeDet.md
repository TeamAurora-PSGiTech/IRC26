# Cone Detection Module Documentation

## Overview
The cone detection system uses YOLOv8 to detect traffic cones in Realsense camera feed, computes 3D positions using depth data, and publishes navigation goals to Nav2 for autonomous rover navigation.

## Components
- `cone_detector.py`: Standalone script for testing YOLO detection on saved images
- `cone_detector_node.py`: ROS2 node for real-time detection, 3D computation, and goal publishing

## Prerequisites
- ROS2 Humble installed
- Realsense SDK and ROS2 wrapper installed
- Python dependencies: ultralytics, opencv-python, cv-bridge, tf2-geometry-msgs
- Workspace built: `colcon build`

## Installation
1. Install ROS2 Humble and Realsense packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-realsense2-camera ros-humble-rmw-cyclonedds-cpp
   ```

2. Install Python dependencies:
   ```bash
   pip install ultralytics opencv-python
   ```

3. Build workspace:
   ```bash
   cd /home/nikash/Documents/IRC26
   source /opt/ros/humble/setup.bash
   colcon build
   ```
## Running the ROS2 Node
### Basic Node Launch
```bash
cd /home/nikash/Documents/IRC26
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomous cone_detector
```

### Full System with Camera and Navigation
1. Launch Realsense camera:
   ```bash
   ros2 launch realsense2_camera rs_launch.py \
     align_depth.enable:=true \
     depth_module.enable:=true \
     depth_module.profile:=640x480x30 \
     pointcloud.enable:=true \
     enable_gyro:=true \
     enable_accel:=true \
     unite_imu_method:=1
   ```

2. Launch Nav2 stack (includes localization and TF):
   ```bash
   cd /home/nikash/Documents/IRC26
   ./scripts/run_nav2_stack.sh
   ```

3. Launch cone detector:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run autonomous cone_detector
   ```

### Using the Convenience Script
The `tmux-start.sh` script can be used to start everything in tmux sessions:
```bash
cd /home/nikash/Documents/IRC26
./tmux-start.sh
```

## Topics
- **Subscribed:**
  - `/camera/camera/color/image_raw`: Color image feed
  - `/camera/camera/depth/image_rect_raw`: Depth image
  - `/camera/camera/depth/camera_info`: Camera intrinsics

- **Published:**
  - `/goal_pose`: Navigation goals (PoseStamped in map frame)
  - `/cone_detector/annotated_image`: Color image with bounding boxes

## Parameters
- Model: `best (2).pt` (YOLOv8 trained on traffic cones)
- Confidence threshold: 0.25
- IoU threshold: 0.6
- Max detections: 10

## Debugging
- Check camera topics: `ros2 topic list | grep camera`
- View detections: `ros2 topic echo /goal_pose`
- Monitor TF: `ros2 run tf2_tools view_frames.py` then `evince frames.pdf`
- View annotated images: `ros2 run image_view image_view image:=/cone_detector/annotated_image`

## Troubleshooting
- **RMW error**: Install `ros-humble-rmw-cyclonedds-cpp`
- **No goals published**: Ensure Nav2 stack is running for TF transforms
- **No detections**: Check camera connection and topics
- **TF extrapolation error**: Fixed in code to use latest transforms

## Configuration Files
- `src/config/nav2_params.yaml`: Nav2 navigation parameters
- `src/config/collision.yaml`: Collision monitoring
- `src/config/ekf.yaml`: EKF localization
- `src/config/navsat.yaml`: GPS integration</content>
<parameter name="filePath">/home/nikash/Documents/IRC26/src/autonomous/autonomous/ConeDet.md
