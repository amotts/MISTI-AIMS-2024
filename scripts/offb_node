#! /usr/bin/env python3

"""
Lightly modified ros node for setting PX4 to offboard mode, commanding basic altitude setpoints.
Modified from original tutorial https://docs.px4.io/main/en/ros/mavros_offboard_python.html
Used for initial system testing and familiarization
"""
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    cmd_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size =10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    start_time = rospy.Time.now()

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0


    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    fly_x = False
    cmd_vel = TwistStamped()
    cmd_vel.twist.linear.x = 1

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        elapsed_time = rospy.Time.now() - start_time
        if elapsed_time.to_sec() < 15:
            pose.pose.position.z = 1
            local_pos_pub.publish(pose)
        elif elapsed_time.to_sec() < 30:
            pose.pose.position.z = 5
            local_pos_pub.publish(pose)
        else:
            start_time = rospy.Time.now()
        #     fly_x = True

        # if fly_x:
        #     cmd_vel_pub.publish(cmd_vel)


        # elif current_state.mode == "OFFBOARD":
        #     offb_set_mode = SetModeRequest()
        #     offb_set_mode.custom_mode = 'AUTO.LOITER'
        #     set_mode_client.call(offb_set_mode)

        rate.sleep()