cmake_minimum_required(VERSION 3.22)
project(ur5_pick_place)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(planning_scene_setup src/planning_scene_setup.cpp)
ament_target_dependencies(planning_scene_setup
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
)

add_executable(plan_pre_pick src/plan_pre_pick.cpp)
ament_target_dependencies(plan_pre_pick
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(move_named_state src/move_named_state.cpp)
ament_target_dependencies(move_named_state
  rclcpp
  moveit_ros_planning_interface
)

add_executable(pick_place_sequence src/pick_place_sequence.cpp)
ament_target_dependencies(pick_place_sequence
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  shape_msgs
  tf2_eigen
  trajectory_msgs
)

add_executable(kinematics_report src/kinematics_report.cpp)
ament_target_dependencies(kinematics_report
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  tf2_eigen
)

install(TARGETS
  planning_scene_setup
  plan_pre_pick
  move_named_state
  pick_place_sequence
  kinematics_report
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/trajectory_profiles.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
