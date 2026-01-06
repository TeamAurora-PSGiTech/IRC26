
**Nav2 Setup — Camera, RTAB-Map, Nav2 & RViz Goal Troubleshooting**

Prerequisites
- **ROS 2 running** and your workspace sourced (e.g. `source install/setup.bash`).
- Camera drivers (RealSense), `rtabmap` and `nav2` packages installed and on your ROS_DOMAIN.
- Recommended: run everything with the same `use_sim_time` setting (typically `false` for live sensors).

1) Start the RealSense camera
```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

2) Start RTAB-Map (visual SLAM)
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=base_link \
  odom_frame_id:=odom \
  publish_tf:=true \
  visual_odometry:=true \
  approx_sync:=true \
  rtabmap_viz:=false \
  delete_db_on_start:=true
```

3) (Optional) Republish topics from RTAB-Map to standard names
If your other stacks expect topics named `/map` and `/odom`, relay them:
```bash
ros2 run topic_tools relay /rtabmap/map /map
ros2 run topic_tools relay /rtabmap/odom /odom
```

4) Publish camera_link -> base_link tf:
```bash
ros2 run tf2_ros static_transform_publisher \                       
  0.15 0.0 0.25 \                            
  0 0 0 \                                                       
  base_link camera_link
```

5) Launch Nav2
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

6) Send a quick test goal from the command line
This verifies Nav2’s action server is working before using RViz:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{ \
  pose: { header: { frame_id: 'map' }, pose: { position: { x: 2.0, y: 1.0, z: 0.0 }, orientation: { z: 0.0, w: 1.0 } } } }"
```

Using RViz2 to set a Nav2 goal (recommended)
- Launch RViz2 (optionally with a saved config): `rviz2`
- In RViz2 set **Fixed Frame** to `map` (top-left `Global Options`).
- Add helpful displays: `TF`, `Map`, `RobotModel`, `Path`, and `PoseArray`/`Global Options` as needed.
- Preferred: add the **Navigation2** panel (the Nav2 RViz plugin). In RViz: `Panels -> Add New Panel -> Navigation2`.
  - The Navigation2 panel provides a direct `Set Goal` / `Cancel` UI that sends goals to the Nav2 action server.
- If the Navigation2 panel/plugin is not available, use the `2D Nav Goal` tool in the RViz toolbar:
  - Click the `2D Nav Goal` icon, then click & drag in the map view to place the goal and orientation.

RViz goal-setting troubleshooting checklist
- Is the Nav2 action server registered?
  - Run `ros2 action list` and confirm `/navigate_to_pose` appears.
- Is the RViz **Fixed Frame** set to `map`? If not, goals are published in the wrong frame.
- Is a TF tree present (map -> odom -> base_link)?
  - Inspect with `ros2 topic echo /tf` (or `ros2 run tf2_tools view_frames.py` if available).
- Is the map topic present and valid? `ros2 topic list` should show `/map` and `ros2 topic echo /map` should produce data.
- Are there any name remappings? If RTAB-Map is publishing `/rtabmap/map` but Nav2 expects `/map`, ensure the relays above are running or remap topics when launching Nav2.
- If RViz publishes a PoseStamped but Nav2 does not accept it, use the CLI `ros2 action send_goal` to verify the action server accepts goals — this isolates RViz from Nav2.

Common fixes when RViz goal-setting fails
- Fixed frame mismatch: set RViz `Fixed Frame` to `map`.
- Missing TF: start a transform between `map` and `odom` (rtabmap usually publishes this). Without it, Nav2 cannot locate the robot.
- Wrong topics: relay `/rtabmap/map` -> `/map` and `/rtabmap/odom` -> `/odom` or remap when launching Nav2.
- Plugin missing: install the Nav2 RViz plugin or use the `2D Nav Goal` tool and verify topics manually.

Future Work:
- Save a working RViz config once your displays and frames are correct — then load it with `rviz2 -d <your_config.rviz>`.
