#!/usr/bin/env python

import rospy
from rosplan_dispatch_msgs.msg import EsterelPlan

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ': %s', data)

def listener():

    rospy.init_node('rosplan_listener', anonymous=False)

    rospy.Subscriber('/rosplan_parsing_interface/complete_plan', EsterelPlan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
