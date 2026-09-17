#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur5_plan_pre_pick",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(5.0);

  geometry_msgs::msg::PoseStamped pre_pick;
  pre_pick.header.frame_id = "base_link";
  pre_pick.pose.position.x = 0.65;
  pre_pick.pose.position.y = -0.30;
  pre_pick.pose.position.z = 0.45;

  // tool0 vertical hacia abajo, conservando la orientación usada en HOME.
  pre_pick.pose.orientation.x = 0.7071067812;
  pre_pick.pose.orientation.y = -0.7071067812;
  pre_pick.pose.orientation.z = 0.0;
  pre_pick.pose.orientation.w = 0.0;

  move_group.setPoseTarget(pre_pick, "tool0");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group.plan(plan));

  if (success) {
    RCLCPP_INFO(
      node->get_logger(),
      "PRE-PICK válido: trayectoria calculada con %zu puntos; no se ejecutó.",
      plan.trajectory.joint_trajectory.points.size());
  } else {
    RCLCPP_ERROR(node->get_logger(), "PRE-PICK no produjo una trayectoria válida.");
  }

  bool execute_motion = false;
  node->get_parameter_or("execute", execute_motion, false);
  bool execution_success = true;
  if (success && execute_motion) {
    RCLCPP_INFO(node->get_logger(), "Ejecutando HOME -> PRE-PICK...");
    execution_success = static_cast<bool>(move_group.execute(plan));
    if (execution_success) {
      RCLCPP_INFO(node->get_logger(), "PRE-PICK alcanzado correctamente.");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Falló la ejecución hacia PRE-PICK.");
    }
  }

  move_group.clearPoseTargets();
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return (success && execution_success) ? 0 : 1;
}
