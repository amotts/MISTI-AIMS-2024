#! /usr/bin/env python3

"""
ROS node for connecting to PX4 autopilot, changing mode to OFFBOARD, and arming vehicle. Node then shutsdown to not interfere with depth controller
For ARDUOPILOT, mode needs to be set to "GUIDED" instead of offboard
"""
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("set_offb")

    autopilot = rospy.get_param("autopilot", "PX4")
    rospy.loginfo("Autopilot set to: " + autopilot)

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    if autopilot == "ArduPilot":
        rospy.wait_for_service("/mavros/cmd/takeoff")
        takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    start_time = rospy.Time.now()

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rospy.logwarn_throttle_identical(2,"No connection to Flight Controller")
        rate.sleep()
    
    rospy.loginfo("Connection Found")
    
    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0

    if autopilot == "PX4":
        #Send a few setpoints before starting
        for _ in range(100):
            if(rospy.is_shutdown()):
                break
            local_pos_pub.publish(pose)
            rate.sleep()

    offb_set_mode = SetModeRequest()

    if autopilot == "ArduPilot":
        mode_str = 'GUIDED'
    elif autopilot == "PX4":
        mode_str = 'OFFBOARD'
    else:
        rospy.logwarn("Invalid autopilot parameter. Killing Node")
        rospy.signal_shutdown("Set Offb closed")
        exit(1)

    offb_set_mode.custom_mode = mode_str

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != mode_str and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo(mode_str + " enabled")
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    rospy.sleep(rospy.Duration(3))
                    if autopilot == "ArduPilot":
                        takeoff_req = CommandTOL()
                        takeoff_req.min_pitch = 0.0
                        takeoff_req.yaw = 0.0
                        takeoff_req.latitude = 0.0
                        takeoff_req.longitude = 0.0
                        takeoff_req.altitude = 5.0

                        if takeoff_client.call(takeoff_req.min_pitch, takeoff_req.yaw, takeoff_req.latitude, takeoff_req.longitude, takeoff_req.altitude).success:
                            rospy.loginfo("Takeoff successful")
                        else:
                            rospy.logerr("Takeoff failed")

                last_req = rospy.Time.now()

        if autopilot == "PX4":
            if not current_state.armed:
                pose.pose.position.z = 5.0
                local_pos_pub.publish(pose)
            else:
                rospy.signal_shutdown("Set Offb closed")

            rate.sleep()