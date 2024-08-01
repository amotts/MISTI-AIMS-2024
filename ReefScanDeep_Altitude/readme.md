# ReefScan Deep Altitude Project
## Overview
The goal of this project was to analyze existing data from the ReefScan deep for ping sonar depth and pressure depth sensors to model and evaluate possible flight altitude, speeds, and flight control methods for autonomous altitude keeping. The data was provided in the format of log files from ReefScan sensor deployments an the questions posed were:
1. What type of real time filtering of the ping data (altitude) can we do to remove the sensor noise and deal with any odd values
2. Using the ping data and a set ability to move vertically what is the distance / speed relationship that means we don’t hit the bottom, target is 2-3 knots and 3-5m.
3. How can we filter the response so that the platform moves slowly and not erratically but still doesn’t hit
4. If we could look ahead how far would we need to look to help both not hitting but also to do a smooth contour
5. What example benthic profiles can we fly against and what types of structure are problematic?

The data was read from the logfiles and compiled into a custom class in python to allow for processing. Filtering was attempted with gaussian, lowpass, and kalman filters. Additional functionalities to discard single extreme values were implemented. A realtime implimentation of the Kalman filter was implemented as another python object class and applied to generate a flight paths using inputted control conditions. Collisions with the ground can then be detected and tallied.

Forward look capabilities was designed to mimic if there was a ping sonar mounted on the vehicle at specified angle, allowing ground data to be generated in advance of the vehicle. First implimentation of this assumes a fixed angle of the vehicle body and thus the sonar. All values are calculated in absolute coordinates from the starting position of the vehicle. Actual implementation will either need relative position to the vehicle or vehicle will need to track it's position relative to the "fixed" origin that is set. e.g. the sonar will give a range. The angle will be known from the mounting angle and vehicle angle and thus a relative depth and distance from the vehicle can be established. Either the ground will need to constantly move towards the vehicle in relative or the vehicle will need to move relative to the static ground.

I did my best to commend and provide documentation throughout the files, especially when defining class methods and functions, but not everything is fully explained and might still have some quirks and hardcodes.

## Method
This project fell into two phases, a data analysis of existing reefscan data, and the development of a depth controller node to interface with the Reefscan autopilot.
### [Data Analysis](Data_Analysis)
I was provided with the photo logs automatically generated from the reefscan units. I compiled logs from the few years worth of field trips and filtered for Reefscan Deep logs with valid data. Data from the photo log csv file is imported into a PhotoLog class object which can then be manipulated with various visulizers and filters. I tested filtering with gaussian smoothing, low pass filtering, and a Bayesian Kalman filter. The Kalman filter had the best performance and the ability to run in real time. I implemented a KalmanFilter class that can perform filtering in real time. The filter also excludes one-off extreme values to account for erroneous readings.

After discussing with Scott, I wrote flight modeling to allow the Kalman filtered flight path to be compared if the vehicle was running at different speeds and altitudes and count instances of collisions. From this I was able to generate a chart of safe altitudes and speeds for each profile to find trends and the sweet spot. However, this assumed perfect response of the vehicle and neglected vehicle dynamics beyond a maximum climb rate.

### [Depth Controller](Reefscan-Feedback-Control)
For the second phase of this project, I attempted to implement a controller that could integrate with the existing Reefscan Deep navigation architecture. From discussion with the team, I determined that the Reefscan used a ROS structure to communicate with an ArduPilot autopilot via MavROS. The controller subscribes to the sensor data, processes it and publishes an altitude setpoint command to MavROS. I wrote a simulator node and an supplementary node to set the vehicle mode to be correct.

I initially just included the Kalman filter in the depth controller with reasonable results. However after speaking with Geoff, he pointed out that even if the vehicle dynamics result in smooth motion without the small magnitude and high frequency variations, the constant setpoint changes will use significant power unecessarily. I then implemented a convolutional filter to provide a pesudo-weighted average for when values did not have large vertical jumps. If a signficant jump is detected, the kernel is turned off to allow a rapid response to the depth change. This produced promisng results over large portions of hisotrical reefscan data. The kernel is customizable and could be refined to provide an optimal response.

Further documentation is provided within the folder itself.

## Uploaded Files
### [Reefscan Feedback Control](Reefscan-Feedback-Control)
This folder contains several ROS nodes and launch scripts that subscribe to and publish MavROS messages for controlling vehicle autopilots in a feedback loop. It contains simulation capabilitiy for SITL for both PX4 and ArduPilot. Simulations were done with quadcopters for ease of implementation. [Depth Controler Node](Reefscan-Feedback-Control/scripts/depth_controller_node.py) is the stand alone node that interfaces with the depth sensors and publishes an altitude control signal. Further documentation is provided within this repo. This is pulled from Open-AIMS and is updated as of the conclusion of my internship on 01.08.24.

### [Depth Graphing](Data_Analysis/DepthGraphing.py)
This was the first file written and served as a testing space for importing the depth data and plotting it. It uses the same import structure as the temperature graphing from the thermal project. Distance is calculated using a haversine function on the Lat and Long and accumulated with a cum sum. The ping depth, pressure depth, and total depth can be plotted.

### [Depth Analysis Functions](Data_Analysis/DepthAnalysisFunctions.py)
This was a major reorganization of the previous depth graphing method with the goal of making it simplified and pythonic. It impiments a class called a Photo Log which contains the data from the photo log and can have various filters and functions applied. This served as the base file where additional capabilities were added to and imported into new files.

### [Depth Analysis](Data_Analysis/DepthAnalysis.py)
The file used to run and use the Photo Log class written in Depth Analysis Functions.

### [Kalman Depth Attempt](Data_Analysis/KalmanDepthAttempt.py)
A first attempt at implementing a Kalman filter on data. Largely derrived from Chapter 4 of [Kalman and Bayesian Filters in Python Textbook](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) by Roger Labbe.

### [Kalman Functions](Data_Analysis/KalmanFunctions.py)
Restructuring of the Kalman filter from Kalman Depth Attempt into a function that can be called. This function is imported into Depth Analysis Functions to allow the application of a Kalman to a Photo Log class object. Was later ammended to include a class object for a realtime Kalman Filter. This runs in real time as new data is collected and inputted and is used in the flight planning and graphing.

### [Collision Analysis](Data_Analysis/CollisionAnalysis.py)
The file used to run and use the createFlightPath and  checkCollisions feature of the photo log class as defined in Depth Analysis Functions. These methods do not work in/simulate real time and are deprecated from the introduction of Flight Modeling Functions.

### [Velocity Analysis](Data_Analysis/Velocity_Analysis.py)
One-off file to find maximum vertical velocities in the Reefscan Deep data to inform the platform capabilities and limtatations for flight planning

### [Flight Modeling Functions](Data_Analysis/FlightModelingFunctions.py)
A restructuring of the createFlightPath and checkCollisions functionalities.Simulates flying along the bottom at a set speed and altitude. Interpolates from the ground signal to get a depth reading and plans/flies a path based on the provided specs and a realtime Kalman filter from Kalman Functions. Checks for total collision points and collision occurences between a flight path and ground and plots them. 

### [Flight Modeling](Data_Analysis/FlightModeling.py)
The file used to run and use the real time flight path creation and check for collisions. One section to run for analyzing a single flight path, and one section for calculating the collisions for a wide range of altitudes and speeds across all photo log depth profiles.

### [Forward Look Functions](Data_Analysis/ForwardLookFunctions.py)
Functions to simulate a forward tilted ping sonar. Inputs of a current position, ground, and sonar angle for a ground data point.

### [Forward Look](Data_Analysis/ForwardLook.py)
The file used to run and test the Forward Look functions.


## Other Files
### MATLAB Online
Two MATLAB files were copied from the Chris Churan's NPS Thesis on remus altitude naviation. 
- remus_run.m
- remusderivalt.m

## Next Steps
Sadly, my time at AIMS has come to an end. However if I had additional time to work on this project, some of my areas of improvement would be (in no particular order):

- Clean up the structure of the nodes, scripts, and py files to make it more streamlined and in line with best practices  
- Continue implementing a forward looking avoidance mechanism, both with a ping sonar and also with a scanning sonar to allow path planning  
- Explore new kernel types and convolutional filtering for better smooth flight performance  
- Update the method that parameters are loaded using yaml file rather than arguments in the launch file  
- Explore other pre-built SITL vehicle models and/or customize one to represent the Reefscan Deep  
- Conduct experiments on the pressure and ping sensors to determine the noise of each sensor to better inform bayesian filtering
- Generate large dataset of artifical ground profiles containing elements similar to what is found in the natural enviornment to run a large quantity of simulations on to evaluate safe operating parameters and danger areas
