#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CameraInfo, Image

#pubimg = rospy.Publisher('/camera/color/image_raw', CameraInfo, queue_size = 10)
pubcam = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size = 10)
def callback(msg):
    caminfo.header = msg.header
    pubcam.publish(caminfo)

subcam = rospy.Subscriber('/camera/color/image_raw', Image, callback)

rospy.init_node('sync')
rospy.loginfo('sync node start')

caminfo = rospy.wait_for_message("bag/camera/color/camera_info", CameraInfo)



rospy.spin()

