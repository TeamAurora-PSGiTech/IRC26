1. Introduction

This project implements a panorama-based site documentation system for IRC26 using ROS 2 and an Intel RealSense depth camera.
The objective is to capture wide-area visual context of exploration sites while recording essential metadata such as orientation, GPS location, elevation, accuracy, and depth context.

The system is designed to be honest, robust, and competition-safe, prioritizing correct engineering practice over unsupported assumptions.

2. System Objectives

The system is developed to:

Capture wide-angle panorama images with a minimum 1:3 height-to-width ratio

Indicate cardinal directions based on rover orientation

Record GPS coordinates, elevation, and accuracy

Provide depth context of the environment

Save all data in a structured, reusable format

Support both single-image and panorama capture modes

3. Hardware and Sensors Used
Sensors

Intel RealSense RGB Camera

Intel RealSense Depth Camera

Intel RealSense IMU

External GPS Module

Processing Platform

ROS 2 (Humble)

Python-based ROS 2 node

4. ROS Topics and Data Sources
Data Type	ROS Topic
RGB Image	/camera/camera/color/image_raw
Depth Image	/camera/camera/depth/image_rect_raw
IMU	/imu/data
GPS Fix	/fix

All metadata is synchronized at capture time.

5. Operating Procedure
TERMINAL 1 — Launch RealSense Camera
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py


This terminal must remain running throughout the operation.

TERMINAL 2 — Run Panorama Node
source /opt/ros/humble/setup.bash
cd ~/IRC26/src/panorama
python3 ros2_panorama.py


A live camera feed window will appear.

6. User Controls
Key	Function
a	Capture single image and save image + metadata
s	Capture one panorama frame (press multiple times while rotating rover)
q	Stitch panorama, save outputs, and exit
7. Data Output and Metadata

Each capture generates:

Image Files

single_image_X.jpg

panorama.jpg

Metadata Files

single_image_X.json

panorama.json

Metadata includes:

Timestamp

IMU yaw and relative cardinal direction

GPS latitude and longitude

Elevation (meters above sea level)

GPS accuracy (meters)

Depth context:

Minimum distance

Maximum distance

Mean distance

Cardinal directions are relative to rover orientation, not magnetic or true north.

8. Current Status and Limitations
Implemented

Panorama image stitching

Relative cardinal direction from IMU

GPS coordinates, elevation, and accuracy

Depth context reporting

Image and JSON data logging

Stable ROS 2 integration

Not Yet Implemented

Absolute image scale (meters per pixel)

At this stage, no absolute scale is claimed in the images or metadata.
Depth information is used only to describe spatial context, not to derive metric measurements.

Scale estimation will be implemented as the final stage, once reference anchoring is introduced and validated.
