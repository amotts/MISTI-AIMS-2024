# Reefscan Deep Feedback Control

This is a collection of ROS nodes and files intended to provide a feedback control mechanism to the Reefscan Deep ROS architecture. The primary goal is to control the altitude of the vehicle using the input from the depth sensors to output setpoint controls to the autopilot. This is simulated using PX4 SITL in Gazebo for ease of testing. However, with minor changes to a few nodes, it is easily transferable to ArduPilot. The MavRos commands are identical and the only difference identified so far is that PX4 'OFFBOARD' mode is the equivalent of ArduPilot 'GUIDED' modes.

test edit to readme