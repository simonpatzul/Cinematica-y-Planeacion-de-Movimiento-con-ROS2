#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr char GROUP_NAME[] = "ur_manipulator";
constexpr char BASE_LINK[] = "base_link";
constexpr char TOOL_LINK[] = "tool0";
constexpr double PI = 3.14159265358979323846;

struct DhResult
{
  Eigen::Isometry3d fk = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
};

std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}

geometry_msgs::msg::Pose downward_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = 0.7071067812;
  pose.orientation.y = -0.7071067812;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}

// Matriz DH modificada: Rx(alfa) Tx(a) Rz(theta) Tz(d).
Eigen::Isometry3d modified_dh(double a, double alpha, double d, double theta)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));
  transform.translate(Eigen::Vector3d(a, 0.0, 0.0));
  transform.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  transform.translate(Eigen::Vector3d(0.0, 0.0, d));
  return transform;
}

DhResult calculate_dh(const Eigen::Matrix<double, 6, 1> & q)
{
  // Parámetros geométricos nominales del UR5 clásico, en metros.
  const double a[6] = {0.0, 0.0, -0.42500, -0.39225, 0.0, 0.0};
  const double alpha[6] = {0.0, PI / 2.0, 0.0, 0.0, PI / 2.0, -PI / 2.0};
  const double d[6] = {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.08230};

  // ROS usa base_link con X e Y invertidos respecto a la base DH del controlador.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ()));

  Eigen::Vector3d origins[6];
  Eigen::Vector3d axes[6];
  for (int i = 0; i < 6; ++i) {
    // En DH modificado, el eje aparece después de Rx(alpha) y Tx(a).
    Eigen::Isometry3d joint_frame = transform;
    joint_frame.rotate(Eigen::AngleAxisd(alpha[i], Eigen::Vector3d::UnitX()));
    joint_frame.translate(Eigen::Vector3d(a[i], 0.0, 0.0));
    origins[i] = joint_frame.translation();
    axes[i] = joint_frame.linear() * Eigen::Vector3d::UnitZ();
    transform = transform * modified_dh(a[i], alpha[i], d[i], q(i));
  }

  DhResult result;
  result.fk = transform;
  const Eigen::Vector3d tool_position = transform.translation();
  for (int i = 0; i < 6; ++i) {
    // Primeras tres filas: velocidad lineal. Últimas tres: velocidad angular.
    result.jacobian.block<3, 1>(0, i) = axes[i].cross(tool_position - origins[i]);
    result.jacobian.block<3, 1>(3, i) = axes[i];
  }
  return result;
}

double orientation_error(const Eigen::Matrix3d & first, const Eigen::Matrix3d & second)
{
  const Eigen::Matrix3d relative = first.transpose() * second;
  const double cosine = std::clamp((relative.trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine);
}

template<typename Derived>
std::string matrix_text(const Eigen::MatrixBase<Derived> & matrix)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(9) << matrix;
  return stream.str();
}

void write_matrix_csv(
  std::ofstream & csv, const std::string & section, const Eigen::MatrixXd & matrix)
{
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      csv << section << ',' << row + 1 << ',' << column + 1 << ','
          << std::setprecision(12) << matrix(row, column) << '\n';
    }
  }
}

bool solve_ik(
  moveit::core::RobotState & state, const moveit::core::JointModelGroup * group,
  const Eigen::Isometry3d & target_in_model_frame)
{
  // El primer intento usa el estado semilla actual. Los siguientes cambian la
  // semilla para no depender de una sola rama de la solución KDL.
  for (int attempt = 1; attempt <= 20; ++attempt) {
    if (attempt > 1) {
      state.setToRandomPositions(group);
      state.update();
    }
    if (state.setFromIK(group, target_in_model_frame, TOOL_LINK, 0.25)) {
      state.update();
      return true;
    }
  }
  return false;
}

bool is_fine_approach(const std::string & state)
{
  return state == "4B" || state == "4D";
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ur5_kinematics_report", options);

  const std::string kinematics = "robot_description_kinematics.ur_manipulator.";
  if (!node->has_parameter(kinematics + "kinematics_solver"))
    node->declare_parameter<std::string>(
      kinematics + "kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  if (!node->has_parameter(kinematics + "kinematics_solver_search_resolution"))
    node->declare_parameter<double>(kinematics + "kinematics_solver_search_resolution", 0.005);
  if (!node->has_parameter(kinematics + "kinematics_solver_timeout"))
    node->declare_parameter<double>(kinematics + "kinematics_solver_timeout", 0.05);

  if (!node->has_parameter("state"))
    node->declare_parameter<std::string>("state", "CURRENT");
  if (!node->has_parameter("results_dir"))
    node->declare_parameter<std::string>("results_dir", "resultados");
  if (!node->has_parameter("qdot"))
    node->declare_parameter<std::vector<double>>(
      "qdot", std::vector<double>{0.05, 0.05, 0.05, 0.05, 0.05, 0.05});
  if (!node->has_parameter("tcp_speed"))
    node->declare_parameter<double>("tcp_speed", -1.0);

  const std::string requested_state = upper(node->get_parameter("state").as_string());
  const std::string results_dir = node->get_parameter("results_dir").as_string();
  const std::vector<double> qdot_parameter = node->get_parameter("qdot").as_double_array();
  double tcp_speed = node->get_parameter("tcp_speed").as_double();

  const std::vector<std::string> valid_states = {
    "HOME", "PICK", "PLACE", "4B", "4D", "CURRENT"};
  if (std::find(valid_states.begin(), valid_states.end(), requested_state) == valid_states.end()) {
    RCLCPP_ERROR(node->get_logger(), "state debe ser HOME, PICK, PLACE, 4B, 4D o CURRENT");
    rclcpp::shutdown();
    return 2;
  }
  if (qdot_parameter.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "El parametro qdot debe contener exactamente 6 valores");
    rclcpp::shutdown();
    return 2;
  }
  if (tcp_speed <= 0.0 && requested_state == "4B") tcp_speed = 0.200;
  if (tcp_speed <= 0.0 && requested_state == "4D") tcp_speed = 0.100;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  MoveGroup move_group(node, GROUP_NAME);
  const moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
  const moveit::core::JointModelGroup * joint_group = robot_model->getJointModelGroup(GROUP_NAME);
  const moveit::core::LinkModel * tool_link = robot_model->getLinkModel(TOOL_LINK);
  if (joint_group == nullptr || tool_link == nullptr || joint_group->getVariableCount() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Grupo ur_manipulator o link tool0 invalido");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::core::RobotStatePtr current = move_group.getCurrentState(3.0);
  moveit::core::RobotState state(robot_model);
  if (current) {
    state = *current;
  } else {
    state.setToDefaultValues();
    state.update();
    if (requested_state == "CURRENT") {
      RCLCPP_ERROR(node->get_logger(), "No se recibio un estado actual del robot");
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_WARN(node->get_logger(), "Sin joint_states; se usa el estado por defecto como semilla");
  }

  if (requested_state == "HOME") {
    const std::map<std::string, double> home = move_group.getNamedTargetValues("HOME");
    if (home.empty()) {
      RCLCPP_ERROR(node->get_logger(), "No existe el estado nombrado HOME en el SRDF");
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }
    for (const auto & value : home) state.setVariablePosition(value.first, value.second);
    state.update();
  } else if (requested_state != "CURRENT") {
    double y = -0.30;
    double z = 0.12;
    if (requested_state == "PLACE" || requested_state == "4D") y = 0.30;
    if (requested_state == "4B" || requested_state == "4D") z = (0.45 + 0.12) / 2.0;

    const geometry_msgs::msg::Pose pose = downward_pose(0.65, y, z);
    Eigen::Isometry3d target_in_base = Eigen::Isometry3d::Identity();
    target_in_base.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    target_in_base.linear() = quaternion.normalized().toRotationMatrix();
    const Eigen::Isometry3d model_to_base = state.getGlobalLinkTransform(BASE_LINK);
    if (!solve_ik(state, joint_group, model_to_base * target_in_base)) {
      RCLCPP_ERROR(node->get_logger(), "KDL/MoveIt no encontro IK para %s", requested_state.c_str());
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }
  }

  Eigen::VectorXd joint_values_dynamic;
  state.copyJointGroupPositions(joint_group, joint_values_dynamic);
  Eigen::Matrix<double, 6, 1> joint_values = joint_values_dynamic;

  const Eigen::Isometry3d base_to_model = state.getGlobalLinkTransform(BASE_LINK).inverse();
  const Eigen::Isometry3d fk_moveit = base_to_model * state.getGlobalLinkTransform(TOOL_LINK);
  const DhResult dh = calculate_dh(joint_values);

  Eigen::MatrixXd jacobian_moveit_dynamic;
  if (!state.getJacobian(
      joint_group, tool_link, Eigen::Vector3d::Zero(), jacobian_moveit_dynamic, false))
  {
    RCLCPP_ERROR(node->get_logger(), "MoveIt no pudo calcular el Jacobiano");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  const Eigen::Matrix<double, 6, 6> jacobian_moveit = jacobian_moveit_dynamic;

  Eigen::Matrix<double, 6, 1> qdot;
  Eigen::Matrix<double, 6, 1> xdot_target = Eigen::Matrix<double, 6, 1>::Zero();
  if (is_fine_approach(requested_state)) {
    // Aproximación vertical hacia abajo. q_dot se obtiene resolviendo J q_dot = x_dot.
    xdot_target(2) = -tcp_speed;
    qdot = jacobian_moveit.colPivHouseholderQr().solve(xdot_target);
  } else {
    for (int i = 0; i < 6; ++i) qdot(i) = qdot_parameter[static_cast<std::size_t>(i)];
  }

  bool joint_velocity_limits_ok = true;
  const std::vector<std::string> joint_names = joint_group->getVariableNames();
  for (int i = 0; i < 6; ++i) {
    const auto & bounds = robot_model->getVariableBounds(joint_names[static_cast<std::size_t>(i)]);
    if (bounds.velocity_bounded_ &&
      (qdot(i) < bounds.min_velocity_ - 1e-9 || qdot(i) > bounds.max_velocity_ + 1e-9))
    {
      joint_velocity_limits_ok = false;
    }
  }

  const Eigen::Matrix<double, 6, 6> jacobian_error = dh.jacobian - jacobian_moveit;
  const Eigen::Matrix<double, 6, 1> xdot_dh = dh.jacobian * qdot;
  const Eigen::Matrix<double, 6, 1> xdot_moveit = jacobian_moveit * qdot;
  const Eigen::Matrix4d fk_error = dh.fk.matrix() - fk_moveit.matrix();

  const double position_difference = (dh.fk.translation() - fk_moveit.translation()).norm();
  const double angle_difference = orientation_error(dh.fk.linear(), fk_moveit.linear());
  const double fk_matrix_max_error = fk_error.cwiseAbs().maxCoeff();
  const double jacobian_max_error = jacobian_error.cwiseAbs().maxCoeff();
  const double jacobian_norm_error = jacobian_error.norm();
  const double cartesian_velocity_error = (xdot_moveit - xdot_target).norm();

  const Eigen::Quaterniond quaternion_moveit(fk_moveit.linear());
  Eigen::Matrix<double, 3, 1> xyz = fk_moveit.translation();
  Eigen::Matrix<double, 4, 1> quaternion;
  quaternion << quaternion_moveit.x(), quaternion_moveit.y(),
    quaternion_moveit.z(), quaternion_moveit.w();

  std::ostringstream report;
  report << std::fixed << std::setprecision(9);
  report << "REPORTE CINEMATICO UR5 - " << requested_state << "\n\n";
  report << "Orden de articulaciones:\n";
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    report << "  q" << i + 1 << " " << joint_names[i] << " = " << joint_values(i) << " rad\n";
  }
  report << "\nPosicion XYZ MoveIt [m]:\n" << matrix_text(xyz.transpose()) << "\n";
  report << "Cuaternion MoveIt [x y z w]:\n" << matrix_text(quaternion.transpose()) << "\n";
  report << "Matriz de rotacion MoveIt:\n" << matrix_text(fk_moveit.linear()) << "\n";
  report << "\nFK MoveIt base_link -> tool0:\n" << matrix_text(fk_moveit.matrix()) << "\n";
  report << "\nFK DH modificado base_link -> tool0:\n" << matrix_text(dh.fk.matrix()) << "\n";
  report << "\nDiferencia T_DH - T_MoveIt:\n" << matrix_text(fk_error) << "\n";
  report << "\nError de posicion [m]: " << position_difference << "\n";
  report << "Error de orientacion [rad]: " << angle_difference << "\n";
  report << "Error maximo de la matriz homogenea: " << fk_matrix_max_error << "\n";
  report << "\nJacobiano analitico DH (filas 1-3 lineales; 4-6 angulares):\n";
  report << matrix_text(dh.jacobian) << "\n";
  report << "\nJacobiano MoveIt/KDL:\n" << matrix_text(jacobian_moveit) << "\n";
  report << "\nDiferencia J_DH - J_MoveIt:\n" << matrix_text(jacobian_error) << "\n";
  report << "\nError maximo Jacobiano: " << jacobian_max_error << "\n";
  report << "Norma Frobenius del error: " << jacobian_norm_error << "\n";
  report << "\nqdot [rad/s]:\n" << matrix_text(qdot.transpose()) << "\n";
  report << "xdot DH [vx vy vz wx wy wz]:\n" << matrix_text(xdot_dh.transpose()) << "\n";
  report << "xdot MoveIt [vx vy vz wx wy wz]:\n" << matrix_text(xdot_moveit.transpose()) << "\n";
  if (is_fine_approach(requested_state)) {
    report << "xdot exigida por perfil [vx vy vz wx wy wz]:\n"
           << matrix_text(xdot_target.transpose()) << "\n";
    report << "Error de velocidad cartesiana: " << cartesian_velocity_error << "\n";
    report << "Limites de velocidad articular: "
           << (joint_velocity_limits_ok ? "CUMPLE" : "NO CUMPLE") << "\n";
  }

  try {
    std::filesystem::create_directories(results_dir);
    const std::string stem = results_dir + "/kinematics_" + requested_state;
    std::ofstream txt(stem + ".txt");
    std::ofstream csv(stem + ".csv");
    if (!txt || !csv) throw std::runtime_error("no se pudieron abrir los archivos de salida");

    txt << report.str();
    csv << "section,row,column,value\n";
    write_matrix_csv(csv, "joint_position_rad", joint_values);
    write_matrix_csv(csv, "position_xyz_m", xyz);
    write_matrix_csv(csv, "quaternion_xyzw", quaternion);
    write_matrix_csv(csv, "rotation_moveit", fk_moveit.linear());
    write_matrix_csv(csv, "fk_moveit", fk_moveit.matrix());
    write_matrix_csv(csv, "fk_dh", dh.fk.matrix());
    write_matrix_csv(csv, "fk_error", fk_error);
    write_matrix_csv(csv, "jacobian_moveit", jacobian_moveit);
    write_matrix_csv(csv, "jacobian_dh", dh.jacobian);
    write_matrix_csv(csv, "jacobian_error", jacobian_error);
    write_matrix_csv(csv, "qdot_rad_s", qdot);
    write_matrix_csv(csv, "xdot_moveit", xdot_moveit);
    write_matrix_csv(csv, "xdot_dh", xdot_dh);
    if (is_fine_approach(requested_state)) write_matrix_csv(csv, "xdot_target", xdot_target);
    csv << "position_error_m,1,1," << position_difference << '\n';
    csv << "orientation_error_rad,1,1," << angle_difference << '\n';
    csv << "fk_matrix_max_error,1,1," << fk_matrix_max_error << '\n';
    csv << "jacobian_max_error,1,1," << jacobian_max_error << '\n';
    csv << "jacobian_frobenius_error,1,1," << jacobian_norm_error << '\n';
    if (is_fine_approach(requested_state)) {
      csv << "tcp_speed_required_m_s,1,1," << tcp_speed << '\n';
      csv << "cartesian_velocity_error,1,1," << cartesian_velocity_error << '\n';
      csv << "joint_velocity_limits_ok,1,1," << (joint_velocity_limits_ok ? 1 : 0) << '\n';
    }
    RCLCPP_INFO(node->get_logger(), "Resultados guardados en %s.[txt|csv]", stem.c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "No se pudieron guardar resultados: %s", error.what());
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "\n%s", report.str().c_str());
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
