#!/usr/bin/env python
# -*- coding: utf-8 -*-
from copy import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image,CameraInfo
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import CommandBool
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
import cv2
toma_imagen_l=False
toma_imagen_r=False
image_lg = Image()
image_rg = Image()
current_local_pose=PoseStamped()

width = 921
height = 691

camera_info_l = CameraInfo()
camera_info_l.header.frame_id="robot_camera_link"
camera_info_l.width = width
camera_info_l.height = height
camera_info_l.distortion_model="plumb_bob"
camera_info_l.D=[0.000263, 0.001106, 0.000199, 0.000115, 0.000000]
camera_info_l.K=[ 795.00626,    0.0    ,  474.67447,
                    0.0    ,  795.31716,  371.14278,
                    0.0    ,    0.0    ,    1.0    ]
camera_info_l.R=[   0.22006244, -0.02428798,  0.97518338,
                    0.06349999,  0.99792634,  0.01052485,
                    -0.9734168,  0.05960801,  0.22114839]
camera_info_l.P=[ 187.87167,     0.0    ,  1600.69669,     0.0    ,
                    0.0    ,   187.87167,   436.35829,     0.0    ,
                    0.0    ,     0.0    ,     1.0    ,     0.0    ]

camera_info_r = CameraInfo()
camera_info_r.header.frame_id="robot_camera_link"
camera_info_r.width = width
camera_info_r.height = height
camera_info_r.distortion_model="plumb_bob"
camera_info_r.D=[-0.003257, 0.003484, -0.000046, 0.000271, 0.000000]
camera_info_r.K=[ 1023.34481,     0.0    ,   571.96364,
                    0.0     ,  1023.70504,   462.99891,
                    0.0     ,     0.0    ,     1.0    ]
camera_info_r.R=[   0.12040627 , -0.10226562,  0.9874432 ,
                    0.06072227 ,  0.99357598,  0.09549645,
                    -0.99086585,  0.04846142,  0.12584258]
camera_info_r.P=[  187.87167,     0.0    ,  1600.69669,  2206.52335,
                    0.0     ,   187.87167,   436.35829,     0.0    ,
                    0.0     ,     0.0    ,     1.0    ,     0.0    ]

def image_cb(msg):
    global toma_imagen_l
    global toma_imagen_r
    global image_lg
    global image_rg
    if (toma_imagen_l==True):
        toma_imagen_l = False
        image_lg = copy(msg)
    if (toma_imagen_r==True):
        toma_imagen_r = False
        image_rg = copy(msg)
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

    image_raw_sub=rospy.Subscriber("iris_gimbal/usb_cam/image_raw", Image, image_cb)
    local_pose_sub=rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pose_cb)

    local_poss_pub = rospy.Publisher("offbnode/pose_local_cmd", PoseStamped, queue_size=10)
    camera_pose_pub=rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=10)

    image_left_pub = rospy.Publisher("offbnode/left/image_raw", Image, queue_size=10)
    image_right_pub = rospy.Publisher("offbnode/right/image_raw", Image, queue_size=10)

    camera_info_left_pub = rospy.Publisher("offbnode/left/camera_info", CameraInfo, queue_size=10)
    camera_info_right_pub = rospy.Publisher("offbnode/right/camera_info", CameraInfo, queue_size=10)
    _cv_bridge=CvBridge()
    rospy.wait_for_service("offbnode/master_ok")

    offboard_client = rospy.ServiceProxy("offbnode/master_ok", CommandBool)
    rospy.loginfo("Esperando servicio offboard")

    while(offboard_client.call(True).success==False):
        rate.sleep()


    camera_pose=MountControl()
    camera_pose.header.frame_id=""
    camera_pose.mode=2
    camera_pose.pitch=-90
    camera_pose.roll=0
    camera_pose.yaw=0
    camera_pose_pub.publish(camera_pose)

    image_l = Image()
    image_r = Image()

    toma_imagen_r= True
    while(not rospy.is_shutdown() and toma_imagen_r):
        rate.sleep()

    while(not rospy.is_shutdown()):

        pose_local=PoseStamped()
        pose_local.pose.position.x = 0
        pose_local.pose.position.y = 0
        pose_local.pose.position.z = 50
        q=quaternion_from_euler(0,0,0)
        pose_local.pose.orientation.x = q[0]
        pose_local.pose.orientation.y = q[1]
        pose_local.pose.orientation.z = q[2]
        pose_local.pose.orientation.w = q[3]

        rospy.loginfo("Ir a posicion 1")
        local_poss_pub.publish(pose_local)
        camera_pose_pub.publish(camera_pose)

        image_rl=_cv_bridge.imgmsg_to_cv2(image_rg, "bgr8")
        image_rl=cv2.resize(image_rl,(width, height))
        image_r2=_cv_bridge.cv2_to_imgmsg(image_rl, "bgr8")

        while(not rospy.is_shutdown() and distancia_to_setpoint(pose_local)>0.05):
            local_poss_pub.publish(pose_local)

            image_l.header.stamp = rospy.Time.now()
            camera_info_l.header.stamp = rospy.Time.now()

            image_r.header.stamp = rospy.Time.now()
            camera_info_r.header.stamp = rospy.Time.now()

            image_left_pub.publish(image_l)
            camera_info_left_pub.publish(camera_info_l)

            image_right_pub.publish(image_r)
            camera_info_right_pub.publish(camera_info_r)

            rate.sleep()

        toma_imagen_l= True
        while(not rospy.is_shutdown() and toma_imagen_l):
            rate.sleep()

        pose_local.pose.position.x = 2
        pose_local.pose.position.y = 0
        pose_local.pose.position.z = 50
        q=quaternion_from_euler(0,0,0)
        pose_local.pose.orientation.x = q[0]
        pose_local.pose.orientation.y = q[1]
        pose_local.pose.orientation.z = q[2]
        pose_local.pose.orientation.w = q[3]

        local_poss_pub.publish(pose_local)
        camera_pose_pub.publish(camera_pose)

        image_ll=_cv_bridge.imgmsg_to_cv2(image_lg, "bgr8")
        image_ll=cv2.resize(image_ll,(width, height))
        image_l2=_cv_bridge.cv2_to_imgmsg(image_ll, "bgr8")

        image_l=copy(image_l2)
        image_r=copy(image_r2)

        rospy.loginfo("Ir a posicion 2")
        while(not rospy.is_shutdown() and distancia_to_setpoint(pose_local)>0.05):
            local_poss_pub.publish(pose_local)

            image_l.header.stamp = rospy.Time.now()
            camera_info_l.header.stamp = rospy.Time.now()

            image_r.header.stamp = rospy.Time.now()
            camera_info_r.header.stamp = rospy.Time.now()

            image_left_pub.publish(image_l)
            camera_info_left_pub.publish(camera_info_l)

            image_right_pub.publish(image_r)
            camera_info_right_pub.publish(camera_info_r)

            rate.sleep()


        toma_imagen_r= True
        while(not rospy.is_shutdown() and toma_imagen_r):
            rate.sleep()



