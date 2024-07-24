#! /usr/bin/env python3

"""
ROS node to complete feedback loop from sensors to autopilot. Subscribes to depth sensor value, runs kalman filter with given parameters, publishes alitude setpoint
Compatible with both PX4 and ArduPilot with in the appropriate mode
"""

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Float32
from KalmanFunctions import *
#from br_msgs.msg import ping1d_distance_simple

class KalmanDepthController:
    def __init__(self):
        # Initialize and get parameters from launch file
        self.initial_depth = rospy.get_param('initial_depth', 5.0)
        self.alt_goal = rospy.get_param('desired_alt', 2.0)
        self.depth_var=rospy.get_param('depth_var', 2)
        self.sensor_var=rospy.get_param('sensor_var', 10)
        self.delta_depth=rospy.get_param('delta_depth', 0)
        self.discard_extreme =rospy.get_param('discard_extreme', True)
        self.upper_extreme=rospy.get_param('upper_extreme', 2)
        self.lower_extreme=rospy.get_param('lower_extreeme', 5)
        self.extreme_lim = rospy.get_param('extreme_lim', 2)

        # Initialize a Kalman Filter using the provided parameters
        self.kf = RealTimeKalman(self.depth_var, self.sensor_var, self.delta_depth, self.discard_extreme, self.upper_extreme, self.lower_extreme, self.extreme_lim)
        self.kf.set_initial_values(self.initial_depth)

        self.command_pose = PoseStamped()
        self.command_pose.header.frame_id = "map"

        # self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.ping_sub = rospy.Subscriber("feedback_loop/ping_depth", Float64, self.ping_rec)
        self.ping_sub = rospy.Subscriber("feedback_loop/pressure_depth", Float32, self.pressure_rec)
        self.cmd_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=2)

        self.alt_delta = None
        self.vehicle_depth = None

    def ping_rec(self, msg):
        # Runs the Kalman filter on the incoming ping depth value and determines the distance from the desired altitude
        self.alt_delta = self.kf.run_kalman(msg.data) - self.alt_goal
        self.publish_alt()
    
    def pressure_rec(self, msg):
        # Stores the pressure depth as the vehicle depth variable
        self.vehicle_depth = msg.data

    def publish_alt(self):
        # Function for publishing an altitude command by applying the difference from desired altitude to the pressure depth to get a new setpoint
        if self.vehicle_depth != None and self.alt_delta != None: 
            self.command_pose.pose.position.z = self.vehicle_depth - self.alt_delta
            self.cmd_pos_pub.publish(self.command_pose)


if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('depth_controller', anonymous=True)
    depth_controller = KalmanDepthController()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down") 