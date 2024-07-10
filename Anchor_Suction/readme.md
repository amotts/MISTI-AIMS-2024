# Anchor Suction

## Overview
This was a brief, two day project with the aim to replicate the plots and equations from "PARAMETRIC METHOD APPLICABLE IN CALCULATING BREAKOUT FORCE AND TIME FOR LIFTING AXISYMMETRIC OBJECTS FROM SEABED" by Jan Michalski (2019). This was posed by Chris Allen. The goal was a plot of force required to pull an anchor off of the sea bed as a function of size and speed. The function required the integration of a ratio of modified Bessel functions over the time history of the value and included numerous physical parameters for the seabed.

Ultimately, a plot was generated that looked similar to the graphs from the paper, however it was not identical to the original and was shifted by about a decade. I was unable to determine the cause for this. It was run for the extrapolated values to complete the exercise however the results were not meaningful since the initial efforts were unable to reproduce the paper results.

## Other Files
### MATLAB Online
The calculations and graphic was all done on MATLAB online (amotz@mit.edu) in the file Anchor_Suction.m. 
An unsucessful attempt at regression was attempted in Anchor_Regression.m