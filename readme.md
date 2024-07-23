# Reefscan Deep Feedback Control

This is a collection of ROS nodes and files intended to provide a feedback control mechanism to the Reefscan Deep ROS architecture. The primary goal is to control the altitude of the vehicle using the input from the depth sensors to output setpoint controls to the autopilot. This is simulated using PX4 SITL in Gazebo for ease of testing. However, with minor changes to a few nodes, it is easily transferable to ArduPilot. The MavRos commands are identical and the only difference identified so far is that PX4 'OFFBOARD' mode is the equivalent of ArduPilot 'GUIDED' modes.

Currently simulated with a quadcopter since the SITL is premade and has similar controls. Some signs may need to be switched when controlling depth below the water. I am not sure whether the Deep autopilot is flying based on a depth as a negative Z value or a positive value increaseing as depth increases.

### Two-Sensor verision

Ground Simulator publishes a depth value that is taken from the autopilot local pose for Z to simulate the vehicle depth from the pressure sensor. The ping depth is calculated as the difference between the interpolated ground and the vehicle depth. Depth setpoint is adjusted by applying the delta from the desired altitude to the previous setpoint altitude.

One concern over this method is error accumulation... Would be potentially good to think about if this is a serious issue and how to reset to ground truth/setpoints