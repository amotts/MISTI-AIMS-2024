#!/usr/bin/env python3

"""
ROS node to simulate the depth sensor data. Currently publishes a single depth value. TODO: ping sonar only
Reads ground profile from CSV file and interpolates points assuming constant x_velocity a specified in parameters
Starts running when vehicel is in OFFBOARD or GUIDED and armed. Publishes at hz rate specified
"""

import rospy
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class GroundSimulator:
    def __init__(self, csv_file, speed):
        self.df = pd.read_csv(csv_file)
        self.speed = speed
        self.start_time = rospy.Time.now().to_sec()

        self.rate = rospy.get_param('pub_rate', 3.0)

        self.positions = self.df['cumulative_distance'].values
        self.depths = self.df['total depth'].values

        self.depth_pub = rospy.Publisher('/feedback_loop/depth_sensor', Float64, queue_size=10) #Should probably publish as a range (CHECK GH Nodes)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped , self.pose_callback)
        
        self.current_ping_depth = 0
        self.current_vehicle_depth = 0
        self.vehicle_hx = [[],[]]

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.positions, self.depths, 'b-')
        self.path, = self.ax.plot([],[], 'r-')
        self.vehicle_marker, = self.ax.plot([], [], 'ro', markersize=10)
        self.ax.set_xlabel('Distance')
        self.ax.set_ylabel('Depth')
        self.ax.set_title('Vehicle Simulation')
        
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        
    def interpolate_depth(self, position):
        return np.interp(position, self.positions, self.depths)

    def pose_callback(self, msg):
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - self.start_time
        current_position = self.speed * elapsed_time
        self.vehicle_hx[0].append(current_position)
        self.current_vehicle_depth = msg.pose.position.z
        self.vehicle_hx[1].append(self.current_vehicle_depth)

    def update_plot(self, frame):
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - self.start_time
        current_position = self.speed * elapsed_time
        self.current_ping_depth = self.interpolate_depth(current_position)
        
        self.depth_pub.publish(self.current_ping_depth)
        
        self.vehicle_marker.set_data([current_position], [self.current_vehicle_depth])
        self.path.set_data(self.vehicle_hx)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()

    def on_close(self, event):
        rospy.signal_shutdown("Plot window closed")

    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot, interval=1.0/self.rate)
        plt.show(block=True)

current_state = State()
run_sim = False

def state_cb(msg):
    if (msg.mode == "OFFBOARD" or msg.mode == 'GUIDED') and msg.armed:
        csv_file = rospy.get_param('csv_file')
        speed = rospy.get_param('speed')
        # rospy.loginfo("Ground Sim Called")
        simulator = GroundSimulator(csv_file, speed)
        # rospy.loginfo("Ground Sim Running")
        simulator.run()

def start_on_offboard():
    rospy.init_node('ground_simulator')
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_on_offboard()
    except rospy.ROSInterruptException:
        pass
