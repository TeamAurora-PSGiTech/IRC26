👉️Landmark Setup — Recording & Visualizing Map Landmarks

💥️This document explains how to record landmarks on the SLAM map and persistently store them using the IRC26 landmark system.

💥️A landmark represents a manually marked point of interest on the map, recorded at the rover’s current position.

👉️Prerequisites

Before running the landmark system, ensure:

💥️ROS 2 Humble is installed

💥️The workspace builds without errors

💥️RTAB-Map and Nav2 are configured correctly

💥️The map → odom → base_link TF tree exists

👉️Full Run — Landmark System

💥️TERMINAL 1 — Launch the Landmark System

cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch realworld_landmark_launch realworld_landmarks.launch.py


👉️This terminal:

💥️Builds the workspace

💥️Launches the full landmark stack

💥️Initializes map, TF, marker visualization, and RViz

⚠️ Keep this terminal running.

💥️TERMINAL 2 — Record Landmarks

Open a new terminal:

cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 run landmark_recorder record_landmark


💥️You should see:

Press 'l' to save landmark

👉️User Control

💥️l - Save a landmark at the current rover position

👉️Each key press:

💥️Reads the current robot position from the map frame

💥️Appends a new landmark to the YAML file

💥️Publishes a marker for visualization

👉️Checking Recorded Landmarks

💥️All landmarks are stored persistently in:

~/ros2_ws/src/marker_demo/config/landmarks.yaml


💥️To view the saved landmarks:

cat ~/ros2_ws/src/marker_demo/config/landmarks.yaml


💥️Coordinates are in meters

💥️Frame reference: map

💥️Landmarks persist across restarts

👉️Notes

💥️Landmarks are manual and operator-defined

💥️No orientation or naming is stored

💥️Landmarks are reloaded automatically when the system restarts

💥️This system is designed for simplicity and reliability

👉️Summary

Using just two terminals, this system allows operators to:

💥️Mark important locations on the map

💥️Store them persistently

💥️Visualize them automatically on restart

