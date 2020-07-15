#!/usr/bin/env python
from pprint import pprint

import rospy
from rosplan_dispatch_msgs.msg import EsterelPlan, ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.msg import StatusUpdate

def callback(data, data_class):
    rospy.loginfo(rospy.get_caller_id() + ': %s', data)
    pprint("Data class: " + str(data_class))
    pprint(data._connection_header)

ROSPLAN_TOPICS = {
    "/rosplan_knowledge_base/status/update": StatusUpdate,
    "/rosplan_parsing_interface/complete_plan": EsterelPlan,
    "/rosplan_plan_dispatcher/action_dispatch": ActionDispatch,
    "/rosplan_plan_dispatcher/action_feedback": ActionFeedback,
}


def main():

    rospy.init_node('rosplan_listener', anonymous=False)

    for topic, data_class in ROSPLAN_TOPICS.items():
        rospy.Subscriber(topic, data_class, callback, data_class)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
