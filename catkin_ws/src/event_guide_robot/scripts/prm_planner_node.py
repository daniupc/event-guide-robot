#!/usr/bin/env python3
"""ROS adapter that enriches semantic guide plans with a PRM waypoint path."""

import json
import math
from pathlib import Path

from prm_planner import NoPathError, load_occupancy_grid_from_yaml, plan_prm_path


def pose_from_amcl(message):
    pose = message.pose.pose
    orientation = pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "yaw": yaw,
    }


def enrich_plan_with_prm_path(plan, start_pose, grid, config):
    path = plan_prm_path(
        grid,
        start=start_pose,
        goal=plan["nav_goal"],
        sample_count=config["sample_count"],
        connection_radius=config["connection_radius_m"],
        random_seed=config["random_seed"],
    )
    enriched = dict(plan)
    enriched["planner"] = "prm"
    enriched["navigation_path"] = path
    return enriched


class PrmPlannerNode:
    """Consume /guide/plan and publish PRM-enriched plans."""

    def __init__(self, rospy_module, string_msg, pose_msg):
        self.rospy = rospy_module
        self.string_msg = string_msg
        self.pose_msg = pose_msg

        default_map = Path(__file__).resolve().parents[1] / "maps" / "map.yaml"
        map_file = self.rospy.get_param("~map_file", str(default_map))
        robot_radius_m = float(self.rospy.get_param("~robot_radius_m", 0.18))
        self.config = {
            "sample_count": int(self.rospy.get_param("~sample_count", 350)),
            "connection_radius_m": float(self.rospy.get_param("~connection_radius_m", 0.75)),
            "random_seed": int(self.rospy.get_param("~random_seed", 13)),
        }
        self.allow_direct_fallback = bool(
            self.rospy.get_param("~allow_direct_fallback", True)
        )
        input_topic = self.rospy.get_param("~input_plan_topic", "/guide/plan")
        output_topic = self.rospy.get_param("~output_plan_topic", "/guide/navigation_plan")
        amcl_topic = self.rospy.get_param("~amcl_pose_topic", "/amcl_pose")

        self.grid = load_occupancy_grid_from_yaml(map_file, robot_radius_m=robot_radius_m)
        self.latest_pose = None

        self.state_pub = self.rospy.Publisher("/guide/state", string_msg, queue_size=10)
        self.result_pub = self.rospy.Publisher("/guide/result", string_msg, queue_size=10)
        self.plan_pub = self.rospy.Publisher(output_topic, string_msg, queue_size=10)
        self.plan_sub = self.rospy.Subscriber(input_topic, string_msg, self.on_plan)
        self.pose_sub = self.rospy.Subscriber(amcl_topic, pose_msg, self.on_pose)

    def publish_text(self, publisher, text):
        publisher.publish(self.string_msg(data=text))

    def on_pose(self, message):
        self.latest_pose = pose_from_amcl(message)

    def _publish_fallback_plan(self, plan, reason):
        fallback = dict(plan)
        fallback["planner"] = "direct_fallback"
        fallback["navigation_path"] = [plan["nav_goal"]]
        fallback["prm_failure_reason"] = reason
        self.publish_text(self.plan_pub, json.dumps(fallback, sort_keys=True))
        self.publish_text(self.result_pub, "PRM no disponible; usando goal directo: {}".format(reason))

    def on_plan(self, message):
        plan = None
        try:
            plan = json.loads(message.data)
            if plan.get("status") != "FOUND":
                raise ValueError("Guide plan status is not FOUND")
            if self.latest_pose is None:
                raise NoPathError("AMCL pose has not been received yet")

            self.publish_text(self.state_pub, "PRM_PLANNING")
            enriched = enrich_plan_with_prm_path(plan, self.latest_pose, self.grid, self.config)
        except (TypeError, ValueError, KeyError, json.JSONDecodeError, NoPathError) as error:
            self.rospy.logwarn("PRM planning failed: %s", error)
            if self.allow_direct_fallback and isinstance(plan, dict) and "nav_goal" in plan:
                try:
                    self._publish_fallback_plan(plan, str(error))
                    self.publish_text(self.state_pub, "PRM_DIRECT_FALLBACK")
                except Exception as fallback_error:  # pragma: no cover - defensive ROS path
                    self.publish_text(self.state_pub, "PRM_PLAN_FAILED")
                    self.publish_text(self.result_pub, "PRM invalido: {}".format(fallback_error))
                return

            self.publish_text(self.state_pub, "PRM_PLAN_FAILED")
            self.publish_text(self.result_pub, "PRM no pudo planificar: {}".format(error))
            return

        self.publish_text(self.plan_pub, json.dumps(enriched, sort_keys=True))
        self.publish_text(self.state_pub, "PRM_PLAN_READY")
        self.publish_text(
            self.result_pub,
            "PRM listo: {} waypoints".format(len(enriched["navigation_path"])),
        )


def main():
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String

    rospy.init_node("prm_planner_node")
    PrmPlannerNode(rospy, String, PoseWithCovarianceStamped)
    rospy.loginfo("prm_planner_node ready")
    rospy.spin()


if __name__ == "__main__":
    main()
