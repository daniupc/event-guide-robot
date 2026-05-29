#!/usr/bin/env python3
"""Send semantic guide plans to ROS move_base.

The module keeps ROS imports inside ``main`` so the pure helpers can be tested in
environments without ROS/catkin installed.
"""

import json
import math


SUCCEEDED_STATUS = 3
DEFAULT_FRAME_ID = "map"


def yaw_to_quaternion(yaw):
    """Return a planar yaw angle as a geometry-compatible quaternion dict."""
    half_yaw = yaw / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


def parse_plan_json(text):
    """Parse a semantic planner JSON message into a dictionary."""
    plan = json.loads(text)
    if not isinstance(plan, dict):
        raise ValueError("Guide plan JSON must contain an object")
    return plan


def _pose_from_plan_or_pose(plan_or_pose):
    if not isinstance(plan_or_pose, dict):
        raise ValueError("Plan or pose must be a dictionary")
    return plan_or_pose.get("nav_goal", plan_or_pose)


def _require_pose_number(pose, key):
    if key not in pose:
        raise ValueError("Navigation pose is missing '{}'".format(key))
    return float(pose[key])


def _resolve_move_base_goal_class(move_base_msg_classes):
    if move_base_msg_classes is not None:
        return move_base_msg_classes.MoveBaseGoal

    from move_base_msgs.msg import MoveBaseGoal

    return MoveBaseGoal


def build_move_base_goal(plan_or_pose, move_base_msg_classes=None, stamp=None):
    """Build a MoveBaseGoal from a planner result or raw pose dictionary.

    ``move_base_msg_classes`` may be a fake namespace with ``MoveBaseGoal`` for
    tests, avoiding any dependency on ROS message packages.
    """
    pose = _pose_from_plan_or_pose(plan_or_pose)
    x = _require_pose_number(pose, "x")
    y = _require_pose_number(pose, "y")
    yaw = float(pose.get("yaw", 0.0))
    frame_id = plan_or_pose.get("frame_id", pose.get("frame_id", DEFAULT_FRAME_ID))
    quaternion = yaw_to_quaternion(yaw)

    move_base_goal_class = _resolve_move_base_goal_class(move_base_msg_classes)
    goal = move_base_goal_class()

    goal.target_pose.header.frame_id = frame_id
    if stamp is not None:
        goal.target_pose.header.stamp = stamp

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = quaternion["x"]
    goal.target_pose.pose.orientation.y = quaternion["y"]
    goal.target_pose.pose.orientation.z = quaternion["z"]
    goal.target_pose.pose.orientation.w = quaternion["w"]
    return goal


class NavigationManagerNode:
    """ROS adapter that consumes /guide/plan and sends goals to move_base."""

    def __init__(self, rospy_module, actionlib_module, string_msg, move_base_action, move_base_goal):
        self.rospy = rospy_module
        self.string_msg = string_msg
        self.move_base_goal = move_base_goal
        self.navigation_timeout_sec = float(
            self.rospy.get_param("~navigation_timeout_sec", 90.0)
        )

        self.state_pub = self.rospy.Publisher("/guide/state", string_msg, queue_size=10)
        self.result_pub = self.rospy.Publisher("/guide/result", string_msg, queue_size=10)
        self.client = actionlib_module.SimpleActionClient("/move_base", move_base_action)

        self.rospy.loginfo("Waiting for /move_base action server")
        self.client.wait_for_server()
        self.rospy.loginfo("Connected to /move_base action server")

        self.plan_sub = self.rospy.Subscriber("/guide/plan", string_msg, self.on_plan)

    def publish_text(self, publisher, text):
        publisher.publish(self.string_msg(data=text))

    def on_plan(self, message):
        try:
            plan = parse_plan_json(message.data)
            if plan.get("status") != "FOUND":
                raise ValueError("Guide plan status is not FOUND")
            goal = build_move_base_goal(
                plan,
                move_base_msg_classes=self,
                stamp=self.rospy.Time.now(),
            )
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.rospy.logerr("Invalid guide plan: %s", error)
            self.publish_text(self.state_pub, "NAVIGATION_FAILED")
            self.publish_text(self.result_pub, "Plan de navegacion invalido: {}".format(error))
            return

        display_name = plan.get("display_name") or plan.get("zone_name") or plan.get("zone_id", "destino")
        self.publish_text(self.state_pub, "NAVIGATE_TO_ZONE")
        self.rospy.loginfo("Sending move_base goal for %s", display_name)
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(
            self.rospy.Duration(self.navigation_timeout_sec)
        )
        if not finished:
            self.client.cancel_goal()
            self.rospy.logwarn(
                "Navigation to %s timed out after %.1f seconds",
                display_name,
                self.navigation_timeout_sec,
            )
            self.publish_text(self.state_pub, "NAVIGATION_FAILED")
            self.publish_text(
                self.result_pub,
                "Navegacion fallida: timeout hacia {}".format(display_name),
            )
            return

        state = self.client.get_state()
        if state == SUCCEEDED_STATUS:
            self.rospy.loginfo("Navigation to %s succeeded", display_name)
            self.publish_text(self.state_pub, "NAVIGATION_SUCCEEDED")
            self.publish_text(
                self.result_pub,
                "Navegacion completada: {}".format(display_name),
            )
        else:
            self.rospy.logwarn("Navigation to %s failed with action state %s", display_name, state)
            self.publish_text(self.state_pub, "NAVIGATION_FAILED")
            self.publish_text(
                self.result_pub,
                "Navegacion fallida hacia {}".format(display_name),
            )

    @property
    def MoveBaseGoal(self):
        """Expose the ROS goal class through the fake-class test seam."""
        return self.move_base_goal


def main():
    import actionlib
    import rospy
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from std_msgs.msg import String

    rospy.init_node("navigation_manager_node")
    NavigationManagerNode(rospy, actionlib, String, MoveBaseAction, MoveBaseGoal)
    rospy.loginfo("navigation_manager_node ready")
    rospy.spin()


if __name__ == "__main__":
    main()
