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
    image_sub = message_filters.Subscriber('/darknet_ros/detection_image', Image)
    depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_image_sub, bb_sub], 10, 0.5)
    ts.registerCallback(callback)
    rospy.spin()

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

def callback(camera_img, depth_img, bb):
  try:
    cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_img, "32FC1")
    cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
    cv_detectionimage = cv_bridge.imgmsg_to_cv2(camera_img)
    #save as png
  except CvBridgeError as e:
    print(e)

  itemto

  objectlist = []
  for i in bb.bounding_boxes:
    objectitem = [] #class, top, bottom, left, right, depth

    x_mean = (i.xmax + i.xmin) / 2
    y_mean = (i.ymax + i.ymin) / 2
    zc = cv_depthimage2[int(y_mean)][int(x_mean)]
    
    objectitem[0] = i.Class
    objectitem[1] = i.ymin
    objectitem[2] = i.ymax
    objectitem[3] = i.xmin
    objectitem[4] = i.xmax
    objectitem[5] = zc
  
  if len(objectlist) == 0:
    return
  elif len(objectlist) == 1:
    main_class = objectlist[0][0]
    x_mean = (objectlist[0][4] + objectlist[0][3]) / 2
    y_mean = (objectlist[0][2] + objectlist[0][1]) / 2
    v1 = np.array(getXYZ(x_mean, y_mean, zc, fx, fy, cx, cy))
    x = v1[0]/1000
    y = v1[1]/1000
    z = v1[2]/1000
    #image_name = 'image_name'
    #boxes = objectlist

  else:
    min_index = 0
    min = objectlist[0][5]
    for i in range len(objectlist):
      if objectlist[i][5] < min:
        min_index = i
        min = objectlist[i][5]

    main_class = objectlist[min_index][0]
    x_mean = (objectlist[min_index][4] + objectlist[min_index][3]) / 2
    y_mean = (objectlist[min_index][2] + objectlist[min_index][1]) / 2
    v1 = np.array(getXYZ(x_mean, y_mean, zc, fx, fy, cx, cy))
    x = v1[0]/1000
    y = v1[1]/1000
    z = v1[2]/1000
    #image_name = 'image_name'
    #boxes = objectlist


    #print(v1/1000, i.Class)
    #we call the closest one main class
    '''
    point_message = PointStamped()
    point_message.header = depth_img.header
    point_message.header.frame_id = "camera_color_optical_frame"
    point_message.point.x = v1[0]/1000
    point_message.point.y = v1[1]/1000
    point_message.point.z = v1[2]/1000
    pub.publish(point_message)
    '''

if __name__ == '__main__':
    main()
