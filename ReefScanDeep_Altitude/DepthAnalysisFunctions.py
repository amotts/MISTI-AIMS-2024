import os
import pandas as pd
import numpy as np
from math import radians, cos, sin, asin, sqrt
from statistics import mean
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.ndimage import gaussian_filter1d
from scipy.signal import butter, lfilter, freqz
from KalmanFunctions import *

def parse_timestamp_from_string(timestamp_str):
    # Parse the timestamp string to a datetime object
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
    return c * r *1000 #returns meters

def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

    # Function to apply the Butterworth low-pass filter to a signal
def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

class GPSError(Exception):
    """Bad GPS data exception class."""
    pass

class MissingDataError(Exception):
    """Missing Data exception class."""
    pass

class SensorIDError(Exception):
    """Wrong sensor type exception class."""
    pass
class TimestampError(Exception):
    """Incorrect Timestamp/Duration"""
    pass

class PhotoLogData:
    """
    A class to represent the data found within a ReefScan photo_log file
    """
    def __init__(self):
        """
        Initializes a instance of a photo log. Most parameters are empty until Data is initialized

        Column ID names need to match the column titles on the csv file to be loaded 
        """
        self.numlog = None
        self.filepath = None
        self.filename = None
        self.df = None
        self.sensor_id = None
        self.timestamp_column = 'filename_string'
        self.sensorID_column = 'device_id'
        self.ping_column = 'ping_depth'
        self.pressure_column = 'pressure_depth'

    
    def __str__(self):
         if self.filename == None:
              return('PhotoLog has not been initialized. Use command initializeData(filepath, numlog) to initialize.')
         else:
            total_time = self.df['Elapsed Time'].iloc[-1]
            return f'PhotoLog data loaded from log number {self.numlog}. A {self.sensor_id} was used to collect {total_time} seconds of data.'

    def initializeData(self, filepath, numlog):
        """
        Reads the data from the specified photo log number from the provided folder path

        Fills out the object parameters for numlog, filepath, filename, sensor_ID, and creates the df

        Additional columns are created in the df for the elapsed time, distance traveled per data point, the cumulative distance traveled, and the total water depth
        """
        self.numlog = numlog
        self.filepath = filepath

        #Find the file with the correct log number and load it
        for filename in os.listdir(self.filepath):
            good_str = "g" + str(self.numlog)+"."
            if good_str in filename:
                self.filename = filename
        try:
            file_path = os.path.join(self.filepath, self.filename)
            self.df = pd.read_csv(file_path)
        
            #Get the sensor ID for this log
            self.sensor_id = self.df[self.sensorID_column].iloc[0]

            # Convert the timestamp to datetime
            self.df[self.timestamp_column] = self.df[self.timestamp_column].apply(parse_timestamp_from_string)
            start_time = self.df[self.timestamp_column].iloc[0]
            self.df['Elapsed Time'] = (self.df[self.timestamp_column] - start_time).dt.total_seconds()

            # Calculate Distance Traveled
            distances = [0]  # Start with 0 for the first point
            for i in range(1, len(self.df)):
                dist = haversine(self.df.loc[i-1, 'latitude'], self.df.loc[i-1, 'longitude'],
                                        self.df.loc[i, 'latitude'], self.df.loc[i, 'longitude'])
                distances.append(dist)
            self.df['distance'] = distances
            self.df['cumulative_distance'] = self.df['distance'].cumsum()

            #Adjust units to be in meters and calculate total depth
            self.df[self.ping_column] = self.df[self.ping_column].apply(lambda x: x/1000)
            self.df['total depth'] = -self.df[self.ping_column]-self.df[self.pressure_column]
        except TypeError:
            print(f'Log file {numlog} does not exist in {filepath} folder. Cannot initialize. Try again with new log file')

    def checkDataQuality(self):
        """
        Runs a check for three types of bad data

        Check that the sensor is either a DEEP or unknown, that the GPS data is good, timestamps are from within 24 hours of start time, 
        and that there are no missing depth data points
        """
        quality = True
        try:
            if any(self.df['latitude']== 0.0):
                raise GPSError("Data has missing GPS data")
        except GPSError as e:
            print(f"The data has the following: {e}")
            quality = False

        try:
            total_rows = len(self.df)
            count_no_data = (self.df['total depth'] > 1).sum()
            percent_missing = 100* count_no_data/total_rows
            if count_no_data > 0:
                raise MissingDataError(f"Data is missing {percent_missing}% of the data")
        except MissingDataError as e:
            print(f"The data has the following: {e}")
            quality = False

        try:
            if not any(string in self.sensor_id for string in ['DEEP', 'Unknown']):
                    raise SensorIDError(f"sensor is a {self.sensor_id}")
        except SensorIDError as e:
            print(f"Wrong sensor. The {e}") 
            quality = False

        try:
            if self.df["Elapsed Time"].iloc[-1] > 86400:
                raise TimestampError(f'File spans {self.df["Elapsed Time"].iloc[-1]} seconds')
        except TimestampError as e:
            print(f'The data is logged from over 24h. {e}')
            quality = False


        if quality:
            print("Finished checking data quality. No issues found!")
        else:
            print("See above issue with data quality.")
    
    def plotMake_dist(self):
        """
        Helper function to add labels and titles for a Depth vs Distance Graph
        """
        plt.xlabel('Distance Traveled')
        plt.ylabel('Depth (m)')
        plt.title('Depth vs Distance Traveled')
        plt.legend()
        plt.grid(True)

    def plotMake_time(self):
        """
        Helper function to add labels and titles for a Depth vs Time Graph
        """
        plt.xlabel('Time (s)')
        plt.ylabel('Depth')
        plt.title('Depth vs Time')
        plt.legend()
        plt.grid(True)

    def plotFFT(self, n=10):
        """
        Plots the FFT of the total depth signal

        Assumes a frequency of 3 Hz

        Plots on figure(n) (default is 10)
        """
        depth_data = self.df['total depth'].values
        fft_result = np.fft.fft(depth_data)
        frequencies = np.fft.fftfreq(len(depth_data), 0.333)
        positive_frequencies = frequencies[:len(depth_data) // 2]
        positive_fft_result = np.abs(fft_result[:len(depth_data) // 2])
        plt.figure(n)
        plt.plot(positive_frequencies, positive_fft_result)
        plt.xlim((-0.01,0.1))
        plt.title('Frequency Spectrum (FFT)')
        plt.xlabel('Frequency')
        plt.ylabel('Magnitude')

    def applyLowpass(self, cutoff):
        """
        Apply a butterworth low pass filter of order 5 to the total depth signal with specified frequency cutoff

        Stores the filtered signal as a new df column for "lowpass"

        Assumes a sampling rate of the average rate over the entire log
        """
        depth_data = self.df['total depth'].values
        t = self.df['Elapsed Time'].values
        fs = len(depth_data)/t[-1] # SAMPLE RATE
        self.df["lowpass"] = butter_lowpass_filter(depth_data, cutoff, fs, order=6)

    def applygaussianFilter(self, sigma = 4, discard_extreme = True, upper_extreme=2, lower_extreme=5):
        """
        Apply a 1-D gaussian smoothing to the total depth signal with the specified sigma value (default is 4)

        Kernel size is 2*r+1 where r is 4*sigma

        Stores the filtered signal as a new df column for "gauss filtered data"

        Optional discard_extreme will remove single data points that lie above upper below lower from the preceeding and succeeding data point\\
        Default upper is 2, default lower is 5
        """
        data = self.df['total depth'].values
        if discard_extreme:
            bad_ind = []
            for ind in range(len(data)-2):
                z_prior = data[ind]
                z = data[ind+1]
                z_next = data[ind+2]

                if (((z>z_prior+upper_extreme) or (z<z_prior-lower_extreme)) and ((z>z_next+upper_extreme) or (z<z_next-lower_extreme))) or (z>-0.1): # Discount data point if point is too much higher or lower than previous or if less than 25% depth of previous point CAN USE ALTITUDE AS CUTOFF FOR VALID DATA
                    bad_ind.append(ind+1)
            for bad in bad_ind:
                data[bad] = mean([data[bad-1], data[bad+1]])
                # print("bad data at index ", bad)
            self.df['gauss filtered data'] = gaussian_filter1d(data, sigma)


        else:
            self.df['gauss filtered data'] = gaussian_filter1d(self.df['total depth'], sigma)

    def applyKalmanFilter(self, depth_var=2, sensor_var=10, delta_depth=0, discard_extreme = True, upper_extreme=2, lower_extreme=5, percent_extreme=0.25, extreme_lim = 2):
        """
        Applies a simple 1D Kalman Filter on the total depth signal.
        
        Stores the filtered signal as a new df column for "Kalman Filtered"
        
        Initial guess is the first measurement. Units assumed to be meters/frame as appropriate

        Discounts data points meeting critera for 'extreme' unless/until 'extreme limit' number in a row are recorded 

        Parameters:
        ------------
        measurements - The signal to be filtered\\
        depth_var - The variance assumed for the depth (Default = 2) \\
        sensor_var - The variance assumed for the depth sensor(s) (Default = 10) \\
        delta_depth - The initial depth rate of change in m per frame (Default = 0) \\
        discard_extreme - Flag to use the following values to remove extreme values (Default is True) \\
        upper_extreme - The bounds for downwards single frame change (Default = 5) \\
        lower_extreme - The bounds for upwards single frame change (Default = 2) \\
        percent_extreme - The percentage of current depth above is considered extreme \\
        extreme_lim - The Number of extreme points ignored before assuming to be valid data \\
        """
        self.df['Kalman Filtered'] = applyKalman(self.df['total depth'].values, depth_var, sensor_var, delta_depth, discard_extreme, upper_extreme, lower_extreme, percent_extreme, extreme_lim)

    def plotTotal(self, n=0, points=False):
        """
        Plot the total depth as a function of elapsed time

        Plots on figure(n) (default is 0)

        optional flag points=True plots in black points instead of colored line
        """
        plt.figure(n)
        if points:
            plt.plot(self.df['Elapsed Time'], self.df['total depth'], 'k.' ,  markersize=2, label="Raw total depth")
        else:
            plt.plot(self.df['Elapsed Time'], self.df['total depth'], label="Raw total depth")
        self.plotMake_time()

    def plotTotal_distance(self, n=0, points=False):
        """
        Plot the total depth as a function of distance

        Plots on figure(n) (default is 0)

        optional flag points=True plots in black points instead of colored line
        """
        plt.figure(n)
        if points:
            plt.plot(self.df['cumulative_distance'], self.df['total depth'], 'k.' ,  markersize=2, label="Raw total depth")
        else:
            plt.plot(self.df['cumulative_distance'], self.df['total depth'], label="Raw total depth")
        self.plotMake_dist()

    def plotPing(self, n=0):
        """
        Plot the ping sonar depth as a function of time

        Plots on figure(n) (default is 0)
        """
        plt.figure(n)
        plt.plot(self.df['Elapsed Time'], -self.df[self.ping_column], label="Ping depth")
        self.plotMake_time()
     
    def plotPing_distance(self, n=0):
        """
        Plot the ping sonar depth as a function of distance

        Plots on figure(n) (default is 0)
        """
        plt.figure(n)
        plt.plot(self.df['cumulative_distance'], -self.df[self.ping_column], label="Ping depth")
        self.plotMake_dist()

    def plotPressure(self, n=0):
        """
        Plot the pressure sensor depth as a function of time

        Plots on figure(n) (default is 0)
        """
        plt.figure(n)
        plt.plot(self.df['Elapsed Time'], -self.df[self.pressure_column], label="Pressure depth")
        self.plotMake_time()

    def plotPressure_distance(self, n=0):
        """
        Plot the pressure sensor depth as a function of distance

        Plots on figure(n) (default is 0)
        """
        plt.figure(n)
        plt.plot(self.df['cumulative_distance'], -self.df[self.pressure_column], label="Pressure depth")
        self.plotMake_dist()

    def plotgaussianFilter(self, n=0):
        """
        Plot the smoothed gaussian total depth signal as a function of time

        Plots on figure(n) (default is 0)

        If the gaussian has not been applied, will print an error
        """
        plt.figure(n)
        try:
            plt.plot(self.df['Elapsed Time'],self.df['gauss filtered data'], label='Gaussian Filtered depth') #  w/ sigma = %f' %sigma)
            self.plotMake_time()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")

    def plotgaussianFilter_distance(self, n=0):
        """
        Plot the smoothed gaussian total depth signal as a function of distance

        Plots on figure(n) (default is 0)

        If the gaussian has not been applied, will print an error
        """
        plt.figure(n)
        try:
            plt.plot(self.df['cumulative_distance'],self.df['gauss filtered data'], label='Gaussian Filtered depth') #  w/ sigma = %f' %sigma)
            self.plotMake_dist()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")
        
    def plotLowPass(self, n=11):
        """
        Plot the lowpass filtered total depth signal as a function of time

        Plots on figure(n) (default is 11)

        If the lowpass has not been applied, will print an error
        """
        plt.figure(n)
        try:
            plt.plot( self.df['Elapsed Time'], self.df['lowpass'], label ='Lowpass')
            self.plotMake_time()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")

    def plotLowPass_distance(self, n=12):
        """
        Plot the lowpass filtered total depth signal as a function of distance

        Plots on figure(n) (default is 12)

        If the lowpass has not been applied, will print an error
        """
        plt.figure(n)
        try:
            plt.plot(self.df['cumulative_distance'], self.df['lowpass'], label ='Lowpass')
            self.plotMake_dist()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")
        
    def plotKalman(self, n=13):
        """
        Plot the Kalman filtered total depth signal as a function of time

        Plots on figure(n) (default is 13)

        If the Kalman has not been applied, will print an error
        """
        plt.figure(n)
        try:
            plt.plot( self.df['Elapsed Time'], self.df['Kalman Filtered'], label ='Kalman Filtered')
            self.plotMake_time()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")

    def plotKalman_distance(self, n=13):
        """
        Plot the Kalman filtered total depth signal as a function of distance

        Plots on figure(n) (default is 13)

        If the Kalman has not been applied, will print an error
        """
        plt.figure(n)
        try:
            plt.plot(self.df['cumulative_distance'], self.df['Kalman Filtered'], label ='Kalman Filtered')
            self.plotMake_dist()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")

    def createFlightPath(self, filter, altitude, max_vel=0.45, print_counter = False):
        """
        DEPRECATED: PLEASE USE STAND ALONE FLIGHT MODELING FUNCTIONS TO MODEL REAL TIME FLIGHT CONTROL

        Creates a simulated vehicle flight path from a filtered total depth signal and altitude.

        Options for filtered total depth are 'Raw', 'Gaussian', 'Lowpass', 'Kalman'

        Altitude - target height above total bottom depth to fly

        max_vel - the maximum absolute vertical speed the vehicle can travel (Default = 0.45 m/s). \\
        Input "None" to remove velocity constraint

        print_counter - boolean for printing number of velocity limited steps (Default is False)
        """
        str_otpions = ['Raw', 'Gaussian', 'Lowpass', 'Kalman']
        filter_data = None

        if filter not in str_otpions:
            print('Option for filter is not valid. Please use a valid input')
            return()

        try:   
            if filter == 'Raw':
                filter_data = self.df['total depth'].values
            elif filter == 'Gaussian':
                filter_data = self.df['gauss filtered data'].values
            elif filter == 'Lowpass':
                filter_data = self.df['lowpass'].values
            elif filter == 'Kalman':
                filter_data = self.df['Kalman Filtered'].values
        
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")
            print("Either apply method to PhotoLog or try again with a different input")
            return()

        flight_data = np.zeros(len(filter_data))
        if max_vel == None:
            flight_data = filter_data + altitude

        else:
            counter = 0
            flight_data[0]=(filter_data[0] + altitude)
            for ind in range(1,len(filter_data)):
                delta_t = self.df["Elapsed Time"].iloc[ind] - self.df["Elapsed Time"].iloc[ind-1]
                if abs((filter_data[ind]+altitude)-flight_data[ind-1])/delta_t > max_vel:
                    flight_data[ind] = flight_data[ind-1] + delta_t * np.sign(filter_data[ind]+altitude-flight_data[ind-1])*max_vel
                    counter += 1
                else:
                    flight_data[ind] = filter_data[ind] + altitude
            if print_counter:
                print(f'Velocity limited {counter} times')

        self.df['flight path']=flight_data

    def plotFlight(self, n=16):
        """
        DEPRECATED

        Plot the Flight Path as a function of time

        Plots on figure(n) (default is 16)

        If the flight path has not been generated, will print an error
        """
        plt.figure(n)
        try:
            plt.plot( self.df['Elapsed Time'], self.df['flight path'], label ='Flight Path')
            self.plotMake_time()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")

    def plotFlight_distance(self, n=16):
        """
        DEPRECATED

        Plot the Flight Path as a function of distance

        Plots on figure(n) (default is 16)

        If the flight path has not been generated, will print an error
        """
        plt.figure(n)
        try:
            plt.plot(self.df['cumulative_distance'], self.df['flight path'], label ='Flight Path')
            self.plotMake_dist()
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")



    def checkCollisions(self, ground, print_all=True, plot_time=False, plot_distance=False, n=15):
        """
        CAUTION: FLIGHT PATH GENERATION USING PHOTOLOG CLASS IS DEPRECATED

        Checks and counts collisions given a ground signal and the flight path.

        Options for ground inputs are 'Raw', 'Gaussian', 'Lowpass', 'Kalman'

        print_all triggers each collision to be printed in the terminal (Defautl is True)

        plot_time and plot_distance triggers the flight path, ground, and collisions to be plotted on the appropriate axis on figure n (Default = 15)

        Returns total number of collisions
        """
        str_otpions = ['Raw', 'Gaussian', 'Lowpass', 'Kalman']

        ground_data = None

        if ground not in str_otpions:
            print('Option for Ground is not valid. Please use a valid input')
            return()

        try: 
            if ground == 'Raw':
                ground_data = self.df['total depth'].values
            elif ground == 'Gaussian':
                ground_data = self.df['gauss filtered data'].values
            elif ground == 'Lowpass':
                ground_data = self.df['lowpass'].values
            elif ground == 'Kalman':
                ground_data = self.df['Kalman Filtered'].values
            
            flight_data = self.df['flight path'].values
                
        
        except KeyError as e:
            print(f"{e} has not been applied to this PhotoLog yet")
            print("Either apply method to PhotoLog or try again with a different input")
            return()

        total_collisions = 0
        col_depths = []
        col_times = []
        col_dist = []
        print(f"Checking collisions between ground ({ground}) and flight path.")
        for ind in range(len(ground_data)): #can this be done by index in without for loop?
            if flight_data[ind] <= ground_data[ind]:
                col_depths.append(ground_data[ind])
                col_times.append(self.df["Elapsed Time"].iloc[ind])
                col_dist.append(self.df['cumulative_distance'].iloc[ind])
                total_collisions += 1
                if print_all:
                    print(f"Collision has occured at {ground_data[ind]} m of depth at time {self.df["Elapsed Time"].iloc[ind]} and distance {self.df['cumulative_distance'].iloc[ind]} m.")
        print(f"Finished collision checking. A total of {total_collisions} were found along the fligh path")
        
            
        if plot_distance:
            plt.figure(n)
            plt.plot(self.df['cumulative_distance'], flight_data, label="Flight Path")
            plt.plot(self.df['cumulative_distance'], ground_data, label="Ground")
            plt.plot(col_dist, col_depths, 'r.' ,  markersize=10, label="Collisions")
            self.plotMake_dist()
        
        if plot_time:
            plt.figure(n)
            plt.plot(self.df['Elapsed Time'], flight_data, label="Flight Path")
            plt.plot(self.df['Elapsed Time'], ground_data, label="Ground")
            plt.plot(col_times, col_depths, 'r.' ,  markersize=10, label="Collisions")
            self.plotMake_time()

        return(total_collisions)