#!/usr/bin/env python
import json
import pickle
import logging
import time

import genpy
import rospy

from std_msgs.msg import String
from actionlib_msgs.msg import GoalID, GoalStatusArray
from rosplan_dispatch_msgs.msg import EsterelPlan, ActionDispatch, ActionFeedback, NonBlockingDispatchActionFeedback, NonBlockingDispatchActionGoal, NonBlockingDispatchActionResult, PlanActionFeedback, PlanActionGoal, PlanActionResult
from rosplan_knowledge_msgs.msg import DomainFormula, StatusUpdate
from rosplan_planning_system.msg import PlannerOutput, ProblemInstance
from visualization_msgs.msg import Marker, MarkerArray


logger = logging.getLogger(__name__)


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
    content = {
        "params": params,
        "start_time": start_time_real,
        "ros_time": start_time_ros,
        "messages": messages
    }
    with open(filepath, "w") as f:
        json.dump(content, f, indent=2, default=encode_ros_datatype)
    logger.info("Session log is saved to: %s", filepath)


def get_current_ros_params():
    params = dict()
    param_names = rospy.get_param_names()
    for name in param_names:
        root_name = name[1:].split("/", 1)[0]
        if root_name in params:
            continue
        params[root_name] = rospy.get_param("/" + root_name)
    return params


def msg_listener(data, data_class):
    connection_header = data._connection_header
    logger.debug("Received message from <%s>", connection_header["topic"])
    messages.append({
        "header": connection_header,
        "data": data,
    })


messages = []
params = get_current_ros_params()
start_time_real = None
start_time_ros = None


def main():
    run_id = params["run_id"]
    rospy.init_node('rosplan_listener', anonymous=False)
    logger.info("Started logging message from session: %s", run_id)

    # recording the current time
    start_time_real = time.time()
    start_time_ros = rospy.get_rostime()
    print(start_time_real, start_time_ros)

    for topic, data_class in ROSPLAN_TOPICS.items():
        logger.debug("Subscribing topic: %s", topic)
        rospy.Subscriber(topic, data_class, msg_listener, data_class)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    finally:
        filepath = "logs/session-%s.json" % run_id
        export_log_json(filepath)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    main()
