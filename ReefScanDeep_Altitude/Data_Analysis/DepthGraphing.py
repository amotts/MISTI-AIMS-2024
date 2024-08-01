import os
import pandas as pd
import numpy as np
from math import radians, cos, sin, asin, sqrt
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.ndimage import gaussian_filter1d
# from haversine import haversine, Unit


def parse_timestamp_from_string(timestamp_str):
    # Example timestamp format: 'YYYYMMDD_HHMMSS_MMM_imnum'
    # Parse the timestamp string to a datetime object
    # print(timestamp_str, type(timestamp_str))
    date_str, time_str, ms_str, im_num = timestamp_str.split('_')  # Split the string into date, time, and milliseconds parts
    full_timestamp_str = date_str + time_str + ms_str  # Concatenate parts
    timestamp = datetime.strptime(full_timestamp_str, '%Y%m%d%H%M%S%f')  # Adjust format as per your string pattern
    return timestamp

def haversine(lat1, lon1, lat2, lon2):
    # Convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # Haversine formula
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371  # Radius of Earth in kilometers. Use 3956 for miles
    return c * r *1000


def plot_depth_data(folder_path, log_num = False, ping = False, combined = False, filtered = False):
    plt.figure(0)  # Create a single figure for all plots

    bad_names = ['g3.','g48','g38','g58','g104','g115','g105']
    good_names = ['g3.'] #,'49','61','62']


    # Iterate through all files in the folder
    for filename in os.listdir(folder_path):
        # print(filename)
        if any(bad in filename for bad in bad_names):
            continue


        if log_num != False:
            good_str = "g" + str(log_num)+"."
            if good_str not in filename:
                continue


        # if not any(good in filename for good in good_names):
        #     continue

        if filename.endswith('.xlsx') or filename.endswith('.csv'):
            file_path = os.path.join(folder_path, filename)
            
            # Read the spreadsheet
            if filename.endswith('.xlsx'):
                df = pd.read_excel(file_path)
            else:
                df = pd.read_csv(file_path)
            
            timestamp_column = 'filename_string'
            sensorID_column = 'device_id'
            ping_column = 'ping_depth'
            pressure_column = 'pressure_depth'

            sensor_id = df[sensorID_column].iloc[0]

            if not any(string in sensor_id for string in ['DEEP', 'Unknown']):
                print(filename, "is not a Deep or Unknown: ", sensor_id)
                continue
            else:
                print(filename, " is a ",sensor_id)

            # Convert the timestamp to datetime
            df[timestamp_column] = df[timestamp_column].apply(parse_timestamp_from_string)
            # Calculate elapsed time in seconds
            start_time = df[timestamp_column].iloc[0]
            df['Elapsed Time'] = (df[timestamp_column] - start_time).dt.total_seconds()

            if any(df['latitude']== 0.0):
                print(filename, "has GPS issues, excluding data")
                continue
            # Calculate Distance Traveled
            distances = [0]  # Start with 0 for the first point
            for i in range(1, len(df)):
                dist = haversine(df.loc[i-1, 'latitude'], df.loc[i-1, 'longitude'],
                                        df.loc[i, 'latitude'], df.loc[i, 'longitude'])
                distances.append(dist)

            # Add the distances to the DataFrame
            df['distance'] = distances
            # Calculate the running total of distances
            df['cumulative_distance'] = df['distance'].cumsum()


            df[ping_column] = df[ping_column].apply(lambda x: x/1000)

            df['total depth'] = -df[ping_column]-df[pressure_column]
            total_rows = len(df)
            count_no_data = (df['total depth'] > 1).sum()
            if count_no_data > total_rows / 2:
                print(filename, "has less than half valid sensor data, excluding")
                continue
            elif count_no_data > 0:
                print (filename, "has %i missing data points" %count_no_data )


            # Plot the data
            if ping:
                plt.plot(df['cumulative_distance'], -df[ping_column], label=filename+ " ping depth") # df[timestamp_column].iloc[0].strftime('%Y-%m-%d %H:%M'))
            if combined:
                plt.plot(df['cumulative_distance'], -df[pressure_column], label=filename+" pressure depth")
                plt.plot(df['cumulative_distance'], df['total depth'], label=filename+ " total depth")
            else:
                plt.plot(df['cumulative_distance'], df['total depth'], label=filename+ " total depth") # 'k.' ,  markersize=2, <--- for use when filtered!


            if filtered:
                sigma = 4
                filtered_depth = gaussian_filter1d(df['total depth'], sigma)
                # plt.figure(0)
                plt.plot(df['cumulative_distance'], filtered_depth, label=filename+ " filtered depth w/ sigma = %f" %sigma)
                plt.xlabel('Distance Traveled')
                plt.ylabel('Total Bottom Depth')
                plt.title('Filtered Depth with sigma = %f' %sigma)
                # plt.xlim([0,1250])
                plt.legend()
                plt.grid(True)

    # Customize and show the plot
    plt.figure(0)
    plt.xlabel('Distance Traveled')
    plt.ylabel('Total Bottom Depth')
    plt.title('Depth vs Distance Traveled')
    # plt.xlim([0,1250])
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage:
folder_path = "Deep_Photo_Logs"
plot_depth_data(folder_path ,ping = False, combined=False, filtered=False)
