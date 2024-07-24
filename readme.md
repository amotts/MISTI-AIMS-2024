# Reefscan Deep Feedback Control

This is a collection of ROS nodes and files intended to provide a feedback control mechanism to the Reefscan Deep ROS architecture. The primary goal is to control the altitude of the vehicle using the input from the depth sensors to output setpoint controls to the autopilot. These nodes are compatible with both PX4 and ArduPilot autopilots as long as the appropriate software is already installed from the manufacturers websites.


Currently simulated with a quadcopter since the SITL is premade and has similar controls. Use of the BlueROV heavy was not sufficient since it lacks responsiveness in the Z direction. Some signs may need to be switched when controlling depth below the water. I am not sure whether the Deep autopilot is flying based on a depth as a negative Z value or a positive value increaseing as depth increases. Vehicle dynamics will be very different so any collision analyis would be deeply flawed, however the controls are the same between platforms.

Ground Simulator publishes a depth value that is taken from the autopilot local pose for Z to simulate the vehicle depth from the pressure sensor. The ping depth is calculated as the difference between the interpolated ground and the vehicle depth. Depth setpoint is adjusted by applying the delta from the desired altitude to the previous setpoint altitude.

One concern over this method is error accumulation... Would be potentially good to think about if this is a serious issue and how to reset to ground truth/setpoints

## Launch Files
Descriptions of launch file purposes and usage included in readme file within launch folder.
### Usage
Ardupilot:
```
roslaunch feedback_control depth_controller_ardu.launch
``` 

PX4:
```  
roslaunch feedback_control depth_controller_sim.launch
```


## Important Nodes
### [Depth Controller](scripts/depth_controller_node.py)
This node subscribes to the pressure and ping depths and publishes a setpoint based on the data being filtered. This is the node that could be inserted inot the existing Reefscan Deep architecture. This uses functions imported from the file KalmanFunctions.py.

### [Ground Simulator](scripts/ground_simulator_node.py)
This node is the simulator and visualizer for the depth controller. It subscribes as necessary to publish simulated pressure and ping depths and provides a visualization of the terrain from a csv file as well as the vehicle location, path history, and setpoint histories.

### [Mode Setter](scripts/set_offb.py)
This node sets the simulated vehicle onto the appropriate mode to allow it to follow the setpoints command of the depth controller. This is modified heavily for the simulated enviornment and probably would not be appropriate to add into the Reefscan architecture. However the autopilot modes will need to be changed somehow before the depth controller can control the vehicle.

## Installation Guides
I used the following resources to install the necessary packages for the simulations:
- MavROS
    - https://masoudir.github.io/mavros_tutorial/
- ArduPilot
    - https://ardupilot.org/dev/docs/where-to-get-the-code.html 
    - https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
- PX4
    - https://docs.px4.io/main/en/ros/mavros_installation.html#ros-melodic-(ubuntu-18.04)
    - https://docs.px4.io/main/en/ros/mavros_offboard_python.html