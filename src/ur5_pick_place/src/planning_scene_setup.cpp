#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
moveit_msgs::msg::CollisionObject make_box(
  const std::string & id, const std::string & frame_id,
  const std::vector<double> & dimensions, double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {dimensions.at(0), dimensions.at(1), dimensions.at(2)};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

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
    "ur5_planning_scene_setup",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  moveit::planning_interface::PlanningSceneInterface planning_scene;

  bool clear_scene = false;
  node->get_parameter_or("clear", clear_scene, false);
  if (clear_scene) {
    planning_scene.removeCollisionObjects({"table", "workpiece", "obstacle"});
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(node->get_logger(), "Objetos retirados temporalmente de la PlanningScene");
    rclcpp::shutdown();
    return 0;
  }

  const std::string frame = "base_link";
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  // Superficie superior de la mesa: z = 0.00 m, al nivel de la base del robot.
  objects.push_back(make_box("table", frame, {0.90, 1.20, 0.20}, 0.55, 0.0, -0.10));

  // Pieza inicial sobre la mesa, en el lado pick (y negativo).
  objects.push_back(make_box("workpiece", frame, {0.05, 0.05, 0.07}, 0.65, -0.30, 0.040));

  // Obstáculo entre pick y place. Se eleva desde la mesa hasta z = 0.40 m.
  objects.push_back(make_box("obstacle", frame, {0.16, 0.08, 0.40}, 0.65, 0.0, 0.20));

  if (!planning_scene.applyCollisionObjects(objects)) {
    RCLCPP_ERROR(node->get_logger(), "No fue posible aplicar los objetos a la PlanningScene");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Escena aplicada: table, workpiece y obstacle en el marco %s", frame.c_str());

  rclcpp::sleep_for(std::chrono::milliseconds(500));
  rclcpp::shutdown();
  return 0;
}
