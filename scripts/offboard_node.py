#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest,CommandBoolResponse, SetMode, SetModeRequest
from roscpp.srv import SetLoggerLevel

current_state = State()
pose = PoseStamped()
pose.header.frame_id = "map"
estado_nodo=CommandBoolResponse()
estado_nodo.success = False
def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global pose
    pose.pose = msg.pose

def offboard_service_cb(req):
    global estado_nodo
    return estado_nodo

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pos_sub = rospy.Subscriber("offbnode/pose_local_cmd", PoseStamped, callback = pose_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("mavros/set_logger_level")
    log_mavros_level_client = rospy.ServiceProxy("mavros/set_logger_level", SetLoggerLevel)

    log_mavros_level_client("ros.mavros_extras","error")

    offboard_service = rospy.Service("offbnode/master_ok", CommandBool,offboard_service_cb)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(30)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    estado_nodo.success = True
                last_req = rospy.Time.now()
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)

        rate.sleep()
