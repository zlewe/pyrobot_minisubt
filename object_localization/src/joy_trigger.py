#!/usr/bin/env python2

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import Joy

def trigger_client():
    rospy.wait_for_service('trigger_save')
    try:
        trig = rospy.ServiceProxy('trigger_save', Trigger)
        req = TriggerRequest()
        res = trig(req)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def joy_cb(msg):
    if msg.buttons[1] == 1:
        #trigger service
        print("Triggering service")
        print("%s" % trigger_client())

def init():
    rospy.Subscriber('joy', Joy, joy_cb, queue_size=1)
    rospy.init_node('joy_trigger_node',anonymous=True)
    rospy.loginfo("Start Trigger Node")
    rospy.spin()

if __name__ == '__main__':
    init()