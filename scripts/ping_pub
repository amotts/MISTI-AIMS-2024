#!/usr/bin/env python3

"""
First attempt at a simulator for the depth sensor publisher.
"""

import rospy
import pandas as pd
import numpy as np
from sensor_msgs.msg import Range
from scipy.interpolate import interp1d
from KalmanFunctions import *

xy_signal = namedtuple('Signal', ['x', 'y'])

def pinger_func(csv_file):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(csv_file)
    ground = xy_signal(df['cumulative_distance'].values, df['total depth'].values)
    return interp1d(ground.x, ground.y)

def ping_pub():
    # Initialize the ROS node
    rospy.init_node('depth_publisher', anonymous=True)
    
    # Load data from CSV and create interpolator
    # csv_file = rospy.get_param('~csv_file', 'depth_data.csv')
    pinger = pinger_func(r'/home/amotz/Documents/Test_Ground_1.csv')
    
    # Create a publisher for depth data
    pub = rospy.Publisher('/feedback_loop/depth_sensor', Range, queue_size=10)
    
    # Set the rate (in Hz) for publishing
    rate = rospy.Rate(3)  # 10 Hz

    #Set Vehcile Velocity
    velocity = 1
    
    # Start time for the simulation
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        # Calculate the current timestamp
        current_time = rospy.Time.now() - start_time
        #rospy.loginfo("ping_pub time: " + str(current_time.to_sec()))
        
        # Interpolate the depth value
        depth_value = pinger(current_time.to_sec()*velocity)
        
        # Create and populate the Range message
        depth_msg = Range()
        depth_msg.header.stamp = rospy.Time.now()
        depth_msg.header.frame_id = 'base_link'
        depth_msg.radiation_type = Range.INFRARED
        depth_msg.field_of_view = 0.1
        depth_msg.min_range = 0.5
        depth_msg.max_range = 25.0
        depth_msg.range = depth_value
        
        # Publish the message
        pub.publish(depth_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        ping_pub()
    except rospy.ROSInterruptException:
        pass