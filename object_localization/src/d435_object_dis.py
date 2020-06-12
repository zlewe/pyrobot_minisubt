#!/usr/bin/env python2

import rospy
import numpy as np
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

pub = rospy.Publisher('/object_pose', PointStamped, queue_size=10)

rospy.init_node('D435_Object_Distance', anonymous=True)
rospy.loginfo("Start D435_Object_Distance")
cv_bridge = CvBridge()

print('Try to get camera info...')

msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo, timeout=None)
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
print('Get camera info')
fx = msg.P[0]
fy = msg.P[5]
cx = msg.P[2]
cy = msg.P[6]

def main():
    depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    ts = message_filters.ApproximateTimeSynchronizer([depth_image_sub, bb_sub], 10, 0.5)
    ts.registerCallback(callback)
    rospy.spin()

def callback(depth_img, bb):
  try:
    cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_img, "32FC1")
    cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
  except CvBridgeError as e:
    print(e)

  for i in bb.bounding_boxes:
    x_mean = (i.xmax + i.xmin) / 2
    y_mean = (i.ymax + i.ymin) / 2

    if i.Class=="bottle":
      rospy.loginfo("see bottle")
      zc = cv_depthimage2[int(y_mean)][int(x_mean)]
      v1 = np.array(getXYZ(x_mean, y_mean, zc, fx, fy, cx, cy))
      print(v1/1000)
      point_message = PointStamped()
      point_message.header = depth_img.header
      point_message.header.frame_id = "camera_color_optical_frame"
      point_message.point.x = v1[0]/1000
      point_message.point.y = v1[1]/1000
      point_message.point.z = v1[2]/1000
      pub.publish(point_message)


def getXYZ(xp, yp, zc, fx,fy,cx,cy):
    #### Definition:
    # cx, cy : image center(pixel)
    # fx, fy : focal length
    # xp, yp: index of the depth image
    # zc: depth
    inv_fx = 1.0/fx
    inv_fy = 1.0/fy
    x = (xp-cx) *  zc * inv_fx
    y = (yp-cy) *  zc * inv_fy
    z = zc
    return (x,y,z)


if __name__ == '__main__':
    main()
