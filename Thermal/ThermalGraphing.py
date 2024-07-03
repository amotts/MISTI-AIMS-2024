import os
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
from ThermalDifferentialGraph import solve_thermal_ivp

def parse_timestamp_from_string(timestamp_str):
    # Example timestamp format: 'YYYYMMDD_HHMMSS_MMM_imnum'
    # Parse the timestamp string to a datetime object
    # print(timestamp_str, type(timestamp_str))
    date_str, time_str, ms_str, im_num = timestamp_str.split('_')  # Split the string into date, time, and milliseconds parts
    full_timestamp_str = date_str + time_str + ms_str  # Concatenate parts
    timestamp = datetime.strptime(full_timestamp_str, '%Y%m%d%H%M%S%f')  # Adjust format as per your string pattern
    return timestamp


def plot_temperature_data(folder_path):
    plt.figure()  # Create a single figure for all plots

    bad_names = ['58','104','63','115','105']
    good_names = ['g3.','49','61','62']


    # Iterate through all files in the folder
    for filename in os.listdir(folder_path):
        # print(filename)
        if any(bad in filename for bad in bad_names):
            continue

        # if not any(good in filename for good in good_names):
            # continue

        if filename.endswith('.xlsx') or filename.endswith('.csv'):
            file_path = os.path.join(folder_path, filename)
            
            # Read the spreadsheet
            if filename.endswith('.xlsx'):
                df = pd.read_excel(file_path)
            else:
                df = pd.read_csv(file_path)
            
            # Assuming the columns are named 'Timestamp' and 'Temperature'
            # Modify these names if your columns are named differently
            timestamp_column = 'filename_string'
            temperature_column = 'sensor_temperature'
            sensorID_column = 'device_id'

            if any(string in df[sensorID_column].iloc[0] for string in ['DEEP', 'Unknown']):
                print(filename, " is a ",df[sensorID_column].iloc[0]," excluding from graph")
                continue

            # Convert the timestamp to datetime
            df[timestamp_column] = df[timestamp_column].apply(parse_timestamp_from_string)
            
            
            # Calculate elapsed time in seconds
            start_time = df[timestamp_column].iloc[0]
            df['Elapsed Time'] = (df[timestamp_column] - start_time).dt.total_seconds()
            # if df['Elapsed Time'].iloc[-1] > 6000:
            #   print(filename, "Exceeds time limit",df['Elapsed Time'].iloc[-1] )

            # if df['Elapsed Time'].max() < 1000:
                # continue
            
            # Plot the data
            plt.plot(df['Elapsed Time'], df[temperature_column], label=filename) # df[timestamp_column].iloc[0].strftime('%Y-%m-%d %H:%M'))
            if df[temperature_column].max() > 65:
              print(filename, "Exceeds Temp limit",df[temperature_column].max())


    initial_T = [30, 35,40,45,50,55,60]
    for T_initial in initial_T:
        sol_t, sol_y = solve_thermal_ivp(T_initial)
        plt.plot(sol_t, sol_y, linestyle='dashed')


    # Customize and show the plot
    plt.xlabel('Elapsed Time (seconds)')
    plt.ylabel('Temperature (Celsius)')
    plt.title('Temperature vs. Elapsed Time')
    # plt.xlim([0,1250])
    # plt.legend()
    plt.grid(True)
    plt.show()

# Example usage:
folder_path = "Photo_Logs"
plot_temperature_data(folder_path)
