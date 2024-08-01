from DepthAnalysisFunctions import*
from KalmanFunctions import *
from FlightModelingFunctions import *

import pandas as pd
import openpyxl

## Individual Log ##
"""
Use this section to visualize a single profile 
"""

log_number = 157
pl = PhotoLogData()
pl.initializeData('Documents', log_number, '/home/amotz')
pl.checkDataQuality()
print(pl.df)
pl.applygaussianFilter(discard_extreme=True)


ground_raw = xy_signal(pl.df['cumulative_distance'].values, pl.df['total depth'].values)
ground_smooth = xy_signal(pl.df['cumulative_distance'].values, pl.df['gauss filtered data'])
flight = xy_signal(*RealTimeFlightPath(ground_raw, 2.0, 1.5, depth_var= 0.5, sensor_var= 10.0, max_y_vel=100))
# checkCollisions(ground_smooth, flight, True)

plt.figure(3)
plt.plot(flight.x, flight.y, label="Flight Path")
plt.plot(ground_raw.x ,ground_raw.y, "k.")
#pl.plotTotal_distance(3, True)
# pl.plotgaussianFilter_distance(3)
plt.show()

## BULK CALCULATION SCRIPT##
"""
Uncomment the code below and run it on its own to generate an excel file with the data for every
log in the logs at each of the speeds and altitudes specified
"""

# # Define the inputs
# logs = [2,57,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171]
# speeds = np.arange(0.5,3,0.5)
# altitudes = np.arange(1,5, 0.5)  # Example altitudes

# # Create a dictionary to hold the data for each ground signal
# data = {}

# for log_num in logs:
#     # Initialize a DataFrame to hold the results for the current ground signal
#     df = pd.DataFrame(index=speeds, columns=altitudes)
    
#     pl = PhotoLogData()
#     pl.initializeData('Deep_Photo_Logs', log_num)
#     pl.applygaussianFilter(discard_extreme=True)
#     ground_raw = xy_signal(pl.df['cumulative_distance'].values, pl.df['total depth'].values)
#     ground_smooth = xy_signal(pl.df['cumulative_distance'].values, pl.df['gauss filtered data'])

#     for speed in speeds:
#         for alt in altitudes:
#             # Calculate the number of collisions
#             flight = xy_signal(*RealTimeFlightPath(ground_raw, x_vel = speed, altitude = alt))
#             collisions = checkCollisions(ground_smooth, flight)
#             # Store the result in the DataFrame
#             df.at[speed, alt] = collisions
            
#     # Store the DataFrame in the dictionary
#     data[f'Photo Log {log_num}'] = df

# # Create a Pandas Excel writer using openpyxl as the engine
# with pd.ExcelWriter('collisions_results.xlsx', engine='openpyxl') as writer:
#     startrow = 0  # Initial row to start writing the tables
#     for sheet_name, df in data.items():
#         worksheet = writer.sheets['Collisions Results'] if 'Collisions Results' in writer.sheets else writer.book.create_sheet('Collisions Results')
#         worksheet.cell(row=startrow + 1, column=1).value = sheet_name
#         # Write the ground signal label
#         df.to_excel(writer, sheet_name='Collisions Results', startrow=startrow)
#         startrow += len(df) + 2  # Move the start row for the next DataFrame
# print("Results saved to collisions_results.xlsx")