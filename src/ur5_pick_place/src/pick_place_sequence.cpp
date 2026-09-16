#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

namespace
{
geometry_msgs::msg::PoseStamped downward_pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.x = 0.7071067812;
  pose.pose.orientation.y = -0.7071067812;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.0;
  return pose;
}

moveit_msgs::msg::CollisionObject workpiece_at_pick()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "base_link";
  object.id = "workpiece";
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.07};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.65;
  pose.position.y = -0.30;
  pose.position.z = 0.040;
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur5_pick_place_sequence",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  MoveGroup move_group(node, "ur_manipulator");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("tool0");
  move_group.setPlanningTime(8.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.20);
  move_group.setMaxAccelerationScalingFactor(0.20);
  move_group.allowReplanning(true);

  // Restablece la pieza por si una ejecución anterior se interrumpió mientras estaba adjunta.
  move_group.detachObject("workpiece");
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  planning_scene.removeCollisionObjects({"workpiece"});
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  if (!planning_scene.applyCollisionObject(workpiece_at_pick())) {
    RCLCPP_ERROR(node->get_logger(), "No se pudo restablecer la pieza en PICK");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  auto plan_and_execute = [&](const std::string & label) {
      move_group.setStartStateToCurrentState();
      MoveGroup::Plan plan;
      bool planned = false;
      for (int attempt = 1; attempt <= 5 && !planned; ++attempt) {
        planned = static_cast<bool>(move_group.plan(plan));
        if (!planned) {
          RCLCPP_WARN(
            node->get_logger(), "%s: reintentando planificación (%d/5)",
            label.c_str(), attempt);
        }
      }
      if (!planned) {
        RCLCPP_ERROR(node->get_logger(), "Falló la planificación: %s", label.c_str());
        return false;
      }
      RCLCPP_INFO(
        node->get_logger(), "%s: plan válido (%zu puntos)", label.c_str(),
        plan.trajectory.joint_trajectory.points.size());
      if (!static_cast<bool>(move_group.execute(plan))) {
        RCLCPP_ERROR(node->get_logger(), "Falló la ejecución: %s", label.c_str());
        return false;
      }
      RCLCPP_INFO(node->get_logger(), "%s: completado", label.c_str());
      return true;
    };

  auto move_named = [&](const std::string & target) {
      move_group.clearPoseTargets();
      if (!move_group.setNamedTarget(target)) {
        RCLCPP_ERROR(node->get_logger(), "No existe el estado %s", target.c_str());
        return false;
      }
      return plan_and_execute(target);
    };

  auto move_pose = [&](const std::string & label, const geometry_msgs::msg::PoseStamped & pose) {
      move_group.clearPoseTargets();
      if (!move_group.setPoseTarget(pose, "tool0")) {
        RCLCPP_ERROR(node->get_logger(), "No se pudo establecer la meta %s", label.c_str());
        return false;
      }
      return plan_and_execute(label);
    };

  const auto pre_pick = downward_pose(0.65, -0.30, 0.45);
  const auto pick = downward_pose(0.65, -0.30, 0.12);
  const auto ready_lift = downward_pose(0.55, 0.15, 0.55);
  const auto over_pick_approach = downward_pose(0.60, -0.18, 0.62);
  // Ruta elevada: la pieza pasa por encima del obstáculo (altura máxima z = 0.40 m).
  const auto over_pick_side = downward_pose(0.60, -0.18, 0.62);
  const auto over_obstacle = downward_pose(0.60, 0.0, 0.62);
  const auto over_place_side = downward_pose(0.60, 0.18, 0.62);
  const auto pre_place = downward_pose(0.65, 0.30, 0.45);
  const auto place = downward_pose(0.65, 0.30, 0.12);

  bool success = move_named("HOME") && move_named("READY") &&
    move_pose("SUBIDA DESDE READY", ready_lift) &&
    move_pose("DESPLAZAMIENTO SOBRE PICK", over_pick_approach) &&
    move_pose("DESCENSO A PRE-PICK", pre_pick) &&
    move_pose("PRE-PICK -> PICK", pick);

  if (success) {
    success = move_group.attachObject(
      "workpiece", "tool0", {"tool0", "flange", "wrist_3_link"});
    if (success) {
      RCLCPP_INFO(node->get_logger(), "PICK: pieza adjuntada a tool0");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    } else {
      RCLCPP_ERROR(node->get_logger(), "No se pudo adjuntar la pieza");
    }
  }

  success = success && move_pose("ELEVACIÓN", pre_pick) &&
    move_pose("SUBIDA SOBRE OBSTÁCULO", over_pick_side) &&
    move_pose("PASO SOBRE OBSTÁCULO", over_obstacle) &&
    move_pose("SALIDA SOBRE OBSTÁCULO", over_place_side) &&
    move_pose("ENTRADA A PRE-PLACE", pre_place) &&
    move_pose("PRE-PLACE -> PLACE", place);

  if (success) {
    success = move_group.detachObject("workpiece");
    if (success) {
      RCLCPP_INFO(node->get_logger(), "PLACE: pieza liberada correctamente");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    } else {
      RCLCPP_ERROR(node->get_logger(), "No se pudo liberar la pieza");
    }
  }

  move_group.clearPoseTargets();
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
