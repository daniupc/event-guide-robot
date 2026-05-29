#!/usr/bin/env python3
"""Resolve user guide commands into semantic zones and stands.

This module is intentionally split into pure functions plus a small ROS wrapper.
The pure functions can be tested without a running ROS master.
"""

import json
import re
from pathlib import Path

import yaml


DEFAULT_NOT_FOUND_REASON = "No zone or stand alias matched the command"


def normalize_text(text):
    """Return a lowercase command with repeated whitespace collapsed."""
    return re.sub(r"\s+", " ", text.strip().lower())


def load_semantic_map(path):
    """Load a semantic map YAML file."""
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def alias_matches(command, aliases):
    """Return True when any normalized alias appears in the command."""
    normalized_command = normalize_text(command)
    return any(normalize_text(alias) in normalized_command for alias in aliases)


def build_found_result(zone_id, zone, label_id=None, label=None):
    """Build the structured plan consumed by downstream guide nodes."""
    if label is None:
        display_name = zone["name"]
        marker_id = None
    else:
        display_name = label.get("display_name", label_id)
        marker_id = label.get("marker_id")

    return {
        "status": "FOUND",
        "zone_id": zone_id,
        "zone_name": zone["name"],
        "label_id": label_id,
        "display_name": display_name,
        "marker_id": marker_id,
        "nav_goal": zone["nav_goal"],
        "search_waypoints": zone["search_waypoints"],
    }


def resolve_command(command, semantic_map):
    """Resolve a user command to a stand first, then to a zone.

    Stand aliases have priority because a command such as "stand Qualcomm"
    should return both the stand and its containing zone. If no stand matches,
    zone aliases are used to allow commands like "vamos a la zona izquierda".
    """
    normalized_command = normalize_text(command)
    zones = semantic_map.get("zones", {})

    for zone_id, zone in zones.items():
        for label_id, label in zone.get("labels", {}).items():
            if alias_matches(normalized_command, label.get("aliases", [])):
                return build_found_result(zone_id, zone, label_id, label)

    for zone_id, zone in zones.items():
        if alias_matches(normalized_command, zone.get("aliases", [])):
            return build_found_result(zone_id, zone)

    return {
        "status": "NOT_FOUND",
        "reason": DEFAULT_NOT_FOUND_REASON,
        "command": normalized_command,
    }


class SemanticPlannerNode:
    """ROS adapter for the semantic planner."""

    def __init__(self, rospy_module, string_msg):
        self.rospy = rospy_module
        self.string_msg = string_msg

        default_map = (
            Path(__file__).resolve().parents[1] / "config" / "semantic_map.yaml"
        )
        semantic_map_path = self.rospy.get_param("~semantic_map", str(default_map))
        self.semantic_map = load_semantic_map(semantic_map_path)

        self.state_pub = self.rospy.Publisher("/guide/state", string_msg, queue_size=10)
        self.result_pub = self.rospy.Publisher("/guide/result", string_msg, queue_size=10)
        self.plan_pub = self.rospy.Publisher("/guide/plan", string_msg, queue_size=10)
        self.command_sub = self.rospy.Subscriber(
            "/guide/command", string_msg, self.on_command
        )

    def publish_text(self, publisher, text):
        publisher.publish(self.string_msg(data=text))

    def on_command(self, message):
        command = message.data
        self.publish_text(self.state_pub, "PARSE_TARGET")
        result = resolve_command(command, self.semantic_map)

        if result["status"] == "FOUND":
            self.publish_text(self.plan_pub, json.dumps(result, sort_keys=True))
            self.publish_text(self.state_pub, "TARGET_RESOLVED")
            self.publish_text(
                self.result_pub,
                "Destino seleccionado: {}".format(result["display_name"]),
            )
            self.rospy.loginfo(
                "Resolved command '%s' to zone=%s label=%s",
                command,
                result["zone_id"],
                result["label_id"],
            )
        else:
            self.publish_text(self.state_pub, "PARSE_FAILED")
            self.publish_text(self.result_pub, result["reason"])
            self.rospy.logwarn("Could not resolve command '%s'", command)


def main():
    import rospy
    from std_msgs.msg import String

    rospy.init_node("semantic_planner_node")
    SemanticPlannerNode(rospy, String)
    rospy.loginfo("semantic_planner_node ready")
    rospy.spin()


if __name__ == "__main__":
    main()
