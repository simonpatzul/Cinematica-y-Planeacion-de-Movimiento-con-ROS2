#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur5_move_named_state",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  std::string target;
  bool execute_motion = false;
  node->get_parameter_or("target", target, std::string("HOME"));
  node->get_parameter_or("execute", execute_motion, false);

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.20);
  move_group.setMaxAccelerationScalingFactor(0.20);

  if (!move_group.setNamedTarget(target)) {
    RCLCPP_ERROR(node->get_logger(), "El estado nombrado '%s' no existe.", target.c_str());
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(move_group.plan(plan));
  if (!planned) {
    RCLCPP_ERROR(node->get_logger(), "No se pudo planificar hacia %s.", target.c_str());
  } else {
    RCLCPP_INFO(
      node->get_logger(), "Plan hacia %s válido: %zu puntos.", target.c_str(),
      plan.trajectory.joint_trajectory.points.size());
  }

  bool executed = true;
  if (planned && execute_motion) {
    executed = static_cast<bool>(move_group.execute(plan));
    if (executed) {
      RCLCPP_INFO(node->get_logger(), "%s alcanzado correctamente.", target.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Falló la ejecución hacia %s.", target.c_str());
    }
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return (planned && executed) ? 0 : 1;
}
