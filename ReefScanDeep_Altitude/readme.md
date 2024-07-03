# ReefScan Deep Altitude Project
## Overview
The goal of this project was to analyze existing data from the ReefScan deep for ping sonar depth and pressure depth sensors to model and evaluate possible flight altitude, speeds, and flight control methods for autonomous altitude keeping. The data was provided in the format of log files from ReefScan sensor deployments an the questions posed were:
1. What type of real time filtering of the ping data (altitude) can we do to remove the sensor noise and deal with any odd values
2. Using the ping data and a set ability to move vertically what is the distance / speed relationship that means we don’t hit the bottom, target is 2-3 knots and 3-5m.
3. How can we filter the response so that the platform moves slowly and not erratically but still doesn’t hit
4. If we could look ahead how far would we need to look to help both not hitting but also to do a smooth contour
5. What example benthic profiles can we fly against and what types of structure are problematic?

The data was read from the logfiles and compiled into a custom class in python to allow for processing. Filtering was attempted with gaussian, lowpass, and kalman filters. Additional functionalities to discard single extreme values were implemented. A realtime implimentation of the Kalman filter was implemented as another python object class and applied to generate a flight paths using inputted control conditions. Collisions with the ground can then be detected and tallied.

I did my best to commend and provide documentation throughout the files, especially when defining class methods and functions, but not everything is fully explained and might still have some quirks and hardcodes.

## Method
TODO: Write explanation for methods and techniques used

## Uploaded Files
### [Depth Graphing](DepthGraphing.py)
This was the first file written and served as a testing space for importing the depth data and plotting it. It uses the same import structure as the temperature graphing from the thermal project. Distance is calculated using a haversine function on the Lat and Long and accumulated with a cum sum. The ping depth, pressure depth, and total depth can be plotted.

### [Depth Analysis Functions](DepthAnalysisFunctions.py)
This was a major reorganization of the previous depth graphing method with the goal of making it simplified and pythonic. It impiments a class called a Photo Log which contains the data from the photo log and can have various filters and functions applied. This served as the base file where additional capabilities were added to and imported into new files.

### [Depth Analysis](DepthAnalysis.py)
The file used to run and use the Photo Log class written in Depth Analysis Functions.

### [Kalman Depth Attempt](KalmanDepthAttempt.py)
A first attempt at implementing a Kalman filter on data. Largely derrived from Chapter 4 of [Kalman and Bayesian Filters in Python Textbook](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) by Roger Labbe.

### [Kalman Functions](KalmanFunctions.py)
Restructuring of the Kalman filter from Kalman Depth Attempt into a function that can be called. This function is imported into Depth Analysis Functions to allow the application of a Kalman to a Photo Log class object. Was later ammended to include a class object for a realtime Kalman Filter. This runs in real time as new data is collected and inputted and is used in the flight planning and graphing.

### [Collision Analysis](CollisionAnalysis.py)
The file used to run and use the createFlightPath and  checkCollisions feature of the photo log class as defined in Depth Analysis Functions. These methods do not work in/simulate real time and are deprecated from the introduction of Flight Modeling Functions.

### [Velocity Analysis](VelocityAnalysis.py)
One-off file to find maximum vertical velocities in the Reefscan Deep data to inform the platform capabilities and limtatations for flight planning

### [Flight Modeling Functions](FlightModelingFunctions.py)
A restructuring of the createFlightPath and checkCollisions functionalities.Simulates flying along the bottom at a set speed and altitude. Interpolates from the ground signal to get a depth reading and plans/flies a path based on the provided specs and a realtime Kalman filter from Kalman Functions. Checks for total collision points and collision occurences between a flight path and ground and plots them.

### [Flight Modeling](FlightModeling.py)
The file used to run and use the real time flight path creation and check for collisions. One section to run for analyzing a single flight path, and one section for calculating the collisions for a wide range of altitudes and speeds across all photo log depth profiles.


## Other Files
### MATLAB Online
Two MATLAB files were copied from the Chris Churan's NPS Thesis on remus altitude naviation. 
- remus_run.m
- remusderivalt.m
