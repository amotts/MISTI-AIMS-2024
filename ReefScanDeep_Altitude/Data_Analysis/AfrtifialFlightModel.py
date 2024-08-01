from DepthAnalysisFunctions import*
from KalmanFunctions import *
from FlightModelingFunctions import *

import pandas as pd
import openpyxl

## Individual Log ##
"""
Use this section to visualize a single profile 
"""

folder_name = "/home/amotz/Documents"
numlog = 1

for filename in os.listdir(folder_name):
    good_str = "profile_" + str(numlog)+"."
    if good_str in filename:
        selected_filename = filename

file_path = os.path.join(folder_name, selected_filename)
df = pd.read_csv(file_path)

ground_raw = xy_signal(df['cumulative_distance'].values, df['total depth'].values)
flight = xy_signal(*RealTimeFlightPath(ground_raw, 2.0, 1.5, depth_var= 0.5, sensor_var= 0.5, max_y_vel=100))

# plt.figure(3)
plt.plot(flight.x, flight.y, label="Flight Path")
plt.plot(ground_raw.x, ground_raw.y, label="Ground")
plt.grid()
plt.show()
