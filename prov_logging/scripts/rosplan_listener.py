#!/usr/bin/env python
import json
import pickle

import genpy
import rospy

from std_msgs.msg import String
from actionlib_msgs.msg import GoalID, GoalStatusArray
from rosplan_dispatch_msgs.msg import EsterelPlan, ActionDispatch, ActionFeedback, NonBlockingDispatchActionFeedback, NonBlockingDispatchActionGoal, NonBlockingDispatchActionResult, PlanActionFeedback, PlanActionGoal, PlanActionResult
from rosplan_knowledge_msgs.msg import DomainFormula, StatusUpdate
from rosplan_planning_system.msg import PlannerOutput, ProblemInstance
from visualization_msgs.msg import Marker, MarkerArray


messages = []


def callback(data, data_class):
    connection_header = data._connection_header
    print("[Sender: %s]" % connection_header["topic"])
    messages.append({
        "header": connection_header,
        "data": data,
    })
    print(
        str(data) + "\n" +
        "--------------------------------------------------------"
    )


ROSPLAN_TOPICS = {
    # "/rosplan_knowledge_base/pddl_action_parameters": DomainFormula,
    "/rosplan_knowledge_base/status/update": StatusUpdate,
    "/rosplan_parsing_interface/complete_plan": EsterelPlan,
    "/rosplan_plan_dispatcher/action_dispatch": ActionDispatch,
    "/rosplan_plan_dispatcher/action_feedback": ActionFeedback,
    "/rosplan_plan_dispatcher/dispatch_plan_action/cancel": GoalID,
    "/rosplan_plan_dispatcher/dispatch_plan_action/feedback": NonBlockingDispatchActionFeedback,
    "/rosplan_plan_dispatcher/dispatch_plan_action/goal": NonBlockingDispatchActionGoal,
    "/rosplan_plan_dispatcher/dispatch_plan_action/result": NonBlockingDispatchActionResult,
    # "/rosplan_plan_dispatcher/dispatch_plan_action/status": GoalStatusArray,
    # "/rosplan_plan_dispatcher/plan_graph": String,
    "/rosplan_planner_interface/planner_output": PlannerOutput,
    "/rosplan_planner_interface/start_planning/cancel": GoalID,
    "/rosplan_planner_interface/start_planning/feedback": PlanActionFeedback,
    "/rosplan_planner_interface/start_planning/goal": PlanActionGoal,
    "/rosplan_planner_interface/start_planning/result": PlanActionResult,
    # "/rosplan_planner_interface/start_planning/status": GoalStatusArray,
    "/rosplan_problem_interface/problem_instance": ProblemInstance,
    "/rosplan_roadmap_server/viz/edges": Marker,
    "/rosplan_roadmap_server/viz/edges_array": MarkerArray,
    "/rosplan_roadmap_server/viz/waypoints": Marker,
    "/rosplan_roadmap_server/viz/waypoints_array": MarkerArray,
}


SUPPORTED_ROS_DATATYPES = (genpy.Message, genpy.rostime.Time, genpy.rostime.Duration)


def encode_message(msg):
    results = dict()
    for slot in msg.__slots__:
        data = getattr(msg, slot)
        results[slot] = encode_ros_datatype(data) if isinstance(data, SUPPORTED_ROS_DATATYPES) else data
    return results


def encode_ros_datatype(o):
    if isinstance(o, genpy.Message):
        return encode_message(o)
    elif isinstance(o, (genpy.rostime.Time, genpy.rostime.Duration)):
        return {"secs": o.secs, "nsecs": o.nsecs}
    else:
        return "[Unsupported type: %s" % type(o)


def export_log_json(filepath):
    with open(filepath, "w") as f:
        json.dump(messages, f, indent=2, default=encode_ros_datatype)


def main():

    rospy.init_node('rosplan_listener', anonymous=False)

    for topic, data_class in ROSPLAN_TOPICS.items():
        rospy.Subscriber(topic, data_class, callback, data_class)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    finally:
        export_log_json("logs/session.json")


if __name__ == '__main__':
    main()
