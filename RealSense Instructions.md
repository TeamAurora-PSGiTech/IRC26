# **Prerequisites**

### **1. Install ROS 2 (Humble recommended)**

Follow the official [ROS 2 installation instructions](https://index.ros.org/doc/ros2/Installation/).

### **2. Install Intel RealSense SDK and ROS 2 Wrapper**

1.  Switch to the root user:
    
    ```bash
    sudo passwd root
    su root
    
    ```
    
2.  Update and upgrade your system:
    
    ```bash
    apt-get update && apt-get upgrade && apt-get dist-upgrade
    
    ```
    
3.  Install required dependencies:
    
    ```bash
    apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
    apt-get install git wget cmake build-essential
    apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
    
    ```
    
4.  Clone the RealSense repository:
    
    ```bash
    git clone https://github.com/realsenseai/librealsense.git
    
    ```
    
5.  Build and install the RealSense SDK:
    
    ```bash
    cd librealsense
    mkdir build && cd build
    cmake ../ -DBUILD_EXAMPLES=true
    make uninstall && make clean && make && make install
    
    ```
    
6.  Set up UDEV rules:
    
    ```bash
    cd ..
    ./scripts/setup_udev_rules.sh
    
    ```
    
7.  Verify the RealSense setup by running the viewer:
    
    ```bash
    realsense-viewer
    
    ```
    

### **3. Install RTAB-Map and ROS 2 packages**

Install the necessary RTAB-Map packages:

```bash
sudo apt install ros-$ROS_DISTRO-rtabmap ros-$ROS_DISTRO-rtabmap-ros

```

### **4. Check Camera Topics**

It is important to confirm the camera topics are available. Run the following command:

```bash
ros2 topic list

```

You should see the following topics:

-   `/camera/camera/color/image_raw`
    
-   `/camera/camera/aligned_depth_to_color/image_raw`
    
-   `/camera/camera/color/camera_info`
    

----------

# **Launch Steps**

### **Step 1 — Start RealSense**

Launch the RealSense camera:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15

```

### **Step 2 — Publish TF between base_link and camera**

This step is necessary for RTAB-Map to function correctly:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link

```

### **Step 3 — Launch RTAB-Map for 2D Mapping**

Launch RTAB-Map with the following command:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=true \
  Rtabmap/DetectionRate:=10 \
  Mem/IncrementalMemory:=false

```

----------

# **Step 4 — Visualize in RViz2**

Start RViz2 to visualize the map:

```bash
rviz2

```

Add the following displays:

-   **Map** → `/map`
    
-   **TF**
    
-   **Odometry** → `/rtabmap/odom`
    
-   **PointCloud2** → `/rtabmap/cloud` (optional)
