#! /usr/bin/env python3

"""
ROS node to complete feedback loop from sensors to autopilot. Subscribes to depth sensor value, runs kalman filter with given parameters, publishes alitude setpoint
Should be compatible with both PX4 and ArduPilot with in the appropriate mode
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from KalmanFunctions import *

class KalmanDepthController:
    def __init__(self):
        self.initial_depth = rospy.get_param('initial_depth', 5.0)
        self.alt_goal = rospy.get_param('desired_alt', 2.0)

        self.depth_var=rospy.get_param('depth_var', 2)
        self.sensor_var=rospy.get_param('sensor_var', 10)
        self.delta_depth=rospy.get_param('delta_depth', 0)
        self.discard_extreme =rospy.get_param('discard_extreme', True)
        self.upper_extreme=rospy.get_param('upper_extreme', 2)
        self.lower_extreme=rospy.get_param('lower_extreeme', 5)
        self.extreme_lim = rospy.get_param('extreme_lim', 2)

        self.kf = RealTimeKalman(self.depth_var, self.sensor_var, self.delta_depth, self.discard_extreme, self.upper_extreme, self.lower_extreme, self.extreme_lim)
        self.kf.set_initial_values(self.initial_depth)

        self.current_state = State()

        self.command_pose = PoseStamped()
        self.command_pose.header.frame_id = "map"

        # self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.ping_sub = rospy.Subscriber("feedback_loop/ping_depth", Float64, self.ping_rec)
        self.ping_sub = rospy.Subscriber("feedback_loop/pressure_depth", Float64, self.pressure_rec)
        self.cmd_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.alt_delta = None
        self.vehicle_depth = None


    # def state_cb(self, msg):
    #     self.current_state = msg

    def ping_rec(self, msg):
        self.alt_delta = self.kf.run_kalman(msg.data) - self.alt_goal
        self.publish_alt()
    
    def pressure_rec(self, msg):
        self.vehicle_depth = msg.data

    def publish_alt(self):
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