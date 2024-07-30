#!/bin/bash

# Navigate to the directory containing sim_vehicle.py if necessary
cd /home/amotz/ardupilot/Tools/autotest || exit

# Run sim_vehicle.py with the desired arguments
python3 sim_vehicle.py -w --out=udp:0.0.0.0:14550 --map --console -v ArduCopter