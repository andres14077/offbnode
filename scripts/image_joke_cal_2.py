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
camera_info_l.header.frame_id="cgo3_camera_joint"
camera_info_l.width = width
camera_info_l.height = height
camera_info_l.distortion_model="plumb_bob"
camera_info_l.D=[0.0003729438545495034, -0.00315317088195925, -0.0004765802708245094, 0.00024373081814096627, 0.0]
camera_info_l.K=[728.7860856090053, 0.0, 458.7760324365459,
                0.0, 729.0444914932343, 344.42387306313896,
                0.0, 0.0, 1.0]
camera_info_l.R=[0.998835484650692, 0.042657079951985456, 0.022539923082986912,
                -0.042633200555106526, 0.999089606249731, -0.0015391212381130818,
                -0.022585057295590606, 0.0005763798465131969, 0.9997447589126074]
camera_info_l.P=[792.208319723304, 0.0, 433.15198135375977, 0.0,
                 0.0, 792.208319723304, 345.5544891357422, 0.0,
                 0.0, 0.0, 1.0, 0.0]

camera_info_r = CameraInfo()
camera_info_r.header.frame_id="cgo3_camera_joint"
camera_info_r.width = width
camera_info_r.height = height
camera_info_r.distortion_model="plumb_bob"
camera_info_r.D=[0.0003939509629667498, -0.00211827827335847, -0.0001372337978665339, 0.0002818090975470498, 0.0]
camera_info_r.K=[730.3494893004635, 0.0, 464.83208769113764, 
                 0.0, 730.5513166023172, 345.914956410221, 
                 0.0, 0.0, 1.0]
camera_info_r.R=[0.9985032387753853, 0.042759891216804555, 0.03410093632428386, 
                -0.042795961562520586, 0.9990837792435064, 0.00032821708034501634, 
                -0.03405565781195663, -0.0017871081779259142, 0.9994183400435253]
camera_info_r.P=[792.208319723304, 0.0, 433.15198135375977, 1690.0204802211429,
                 0.0, 792.208319723304, 345.5544891357422, 0.0,
                 0.0, 0.0, 1.0, 0.0]

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

        pose_local.pose.position.x = 0
        pose_local.pose.position.y = 2
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



