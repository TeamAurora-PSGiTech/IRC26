✅ Launch (3 terminals)

Terminal 1

ros2 run joy joy_node --ros-args -p dev:=/dev/input/event17


Terminal 2

ros2 run joystick_teleop joy_twist

terminal 3

ros2 topic echo /cmd_vel --once
