## Reefscan Deep Feedback Control Launch Files

### [depth_controller_sim.launch](depth_controller_sim.launch)
Full launch file for PX4 simulation of depth following
- Requires PX4 to be installed and on path
- Requires world file to be specified (custom.world is not a standard file but any world file will work)
- Path to CSV file with terrain will need to be changed to appropriate path

### [depth_controller_ardu.launch](depth_controller_ardu.launch)
Full launch file for ArduPilot simulation of depth following
- Requires ardupilot to be installed
- Utilizes start_ardu_sim.sh to start the ardupilot arducopter simulator
    - Path to ArduPilot will need to be changed to appropriate path
- ArduPilot simulator will take ~25 seconds to initialize. Wait until console says GPS status is good

### [depth_controller_sim_PhotoLog.launch](depth_controller_sim_PhotoLog.launch)
Slightly janky version of depth controller sim for PX4 that takes input as a photo log direct from reefscan log file
- Has some unmaintained hardcoded elements for how to find the log file. File path hardcode will need to be updated to use and file must end with g##.csv where number is the paramter log_num specified.

### [ground_sim.launch](ground_sim.launch)
DEPRECATED  
Launch file for starting PX4 simulation in offboard mode to test visualization in ground_simulator_node.py  
No feedback control implemented 

### [start_offb.launch](start_offb.launch)
DEPRECATED  
Original Launch file from tutorial for setting PX4 simulation to OFFBOARD 

### [start_filter_offb.launch](start_filter_offb.launch)
DEPRECATED  
Launch file from testing controlling setpoint via a subscribed topic