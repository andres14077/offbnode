#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest,CommandBoolResponse, SetMode, SetModeRequest
from roscpp.srv import SetLoggerLevel

class Offboard_Master:
    def __init__(self):
        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(30)

        self.current_state = State()
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.estado_nodo=CommandBoolResponse()
        self.estado_nodo.success = False

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.pos_sub = rospy.Subscriber("offbnode/pose_local_cmd", PoseStamped, self.pose_cb)
        rospy.Subscriber("mavros/global_position/local", Odometry, self.tf_callback, queue_size=5)

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.wait_for_service("mavros/set_logger_level")
        self.log_mavros_level_client = rospy.ServiceProxy("mavros/set_logger_level", SetLoggerLevel)

        self.log_mavros_level_client("ros.mavros_extras","error")

        self.offboard_service = rospy.Service("offbnode/master_ok", CommandBool, self.offboard_service_cb)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

        # Send a few setpoints before starting
        for i in range(100):
            if(rospy.is_shutdown()):
                break
            self.pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        self.last_req = rospy.Time.now()
        self.estado_nodo.success = True
        rospy.loginfo("Offboard Master init")

    def state_cb(self,msg):
        self.current_state = msg

    def pose_cb(self,msg):
        self.pose.pose = msg.pose

    def offboard_service_cb(self,req):
        return self.estado_nodo

    def tf_callback(self,msg):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        br.sendTransform(t)

    def update(self):
        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(1)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            self.last_req = rospy.Time.now()
        else:
            if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(1)):
                if(self.arming_client.call(self.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    self.estado_nodo.success = True
                self.last_req = rospy.Time.now()
        self.pose.header.stamp = rospy.Time.now()
        self.local_pos_pub.publish(self.pose)

if __name__ == "__main__":
    rospy.init_node("offb_node_py", anonymous=True)
    nodo=Offboard_Master()
    while(not rospy.is_shutdown()):
        nodo.update()
        nodo.rate.sleep()
