#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image,CameraInfo
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from tf.transformations import quaternion_from_euler
toma_imagen_l=False
toma_imagen_r=False
image_1 = Image()
image_2 = Image()
current_local_pose=PoseStamped()
camera_info_b=CameraInfo()
def image_cb(msg):
    global toma_imagen_l
    global toma_imagen_r
    global image_1
    global image_2
    if (toma_imagen_l==True):
        toma_imagen_l = False
        image_1 =msg
    if (toma_imagen_r==True):
        toma_imagen_r = False
        image_2 =msg
def camera_info_cb(msg):
    global camera_info_b
    camera_info_b=msg
def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg
def distancia_to_setpoint(pose):
    global current_local_pose
    d=(current_local_pose.pose.position.x-pose.pose.position.x)*(current_local_pose.pose.position.x-pose.pose.position.x)
    d+=(current_local_pose.pose.position.y-pose.pose.position.y)*(current_local_pose.pose.position.y-pose.pose.position.y)
    d+=(current_local_pose.pose.position.z-pose.pose.position.z)*(current_local_pose.pose.position.z-pose.pose.position.z)
    return math.sqrt(d)
if __name__ == "__main__":
    rospy.init_node("image_joke_py")

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    rospy.Subscriber("iris_gimbal/usb_cam/image_raw", Image, image_cb)
    rospy.Subscriber("iris_gimbal/usb_cam/camera_info", CameraInfo, camera_info_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pose_cb)

    local_poss_pub = rospy.Publisher("offbnode/pose_local_cmd", PoseStamped, queue_size=10)
    camera_pose_pub=rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=10)

    image_left_pub = rospy.Publisher("offbnode/left/image_raw", Image, queue_size=10)
    image_right_pub = rospy.Publisher("offbnode/right/image_raw", Image, queue_size=10)

    camera_info_left_pub = rospy.Publisher("offbnode/left/camera_info", CameraInfo, queue_size=10)
    camera_info_right_pub = rospy.Publisher("offbnode/right/camera_info", CameraInfo, queue_size=10)

    rospy.wait_for_service("offbnode/master_ok")

    offboard_client = rospy.ServiceProxy("offbnode/master_ok", CommandBool)

    while(offboard_client.call(True).success==False):
        rate.sleep()

    pose_local=PoseStamped()
    pose_local.pose.position.x = 0
    pose_local.pose.position.y = 0
    pose_local.pose.position.z = 50
    q=quaternion_from_euler(0,0,math.atan2(1,0))
    pose_local.pose.orientation.x = q[0]
    pose_local.pose.orientation.y = q[1]
    pose_local.pose.orientation.z = q[2]
    pose_local.pose.orientation.w = q[3]

    camera_pose=MountControl()
    camera_pose.header.frame_id="map"
    camera_pose.mode=2
    camera_pose.pitch=-90

    while(not rospy.is_shutdown() and distancia_to_setpoint(pose_local)>0.05):
        local_poss_pub.publish(pose_local)
        camera_pose_pub.publish(camera_pose)
        rate.sleep()
    toma_imagen_l= True
    pose_1=current_local_pose.pose
    while(not rospy.is_shutdown() and toma_imagen_l):
        rate.sleep()

    pose_local.pose.position.x = 2
    pose_local.pose.position.y = 0
    pose_local.pose.position.z = 50
    q=quaternion_from_euler(0,0,math.atan2(1,0))
    pose_local.pose.orientation.x = q[0]
    pose_local.pose.orientation.y = q[1]
    pose_local.pose.orientation.z = q[2]
    pose_local.pose.orientation.w = q[3]

    while(not rospy.is_shutdown() and distancia_to_setpoint(pose_local)>0.05):
        local_poss_pub.publish(pose_local)
        camera_pose_pub.publish(camera_pose)
        rate.sleep()
    toma_imagen_r= True
    pose_2=current_local_pose.pose
    while(not rospy.is_shutdown() and toma_imagen_r):
        rate.sleep()