cmake_minimum_required(VERSION 3.0.2)
project(event_guide_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/semantic_planner_node.py
  scripts/navigation_manager_node.py
  scripts/local_search_manager_node.py
  scripts/vision_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
