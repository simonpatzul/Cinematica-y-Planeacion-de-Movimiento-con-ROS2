#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr int INTENTOS = 10;
constexpr double TOL_POS = 0.015;
constexpr double TOL_ORI = 0.08;
// NOLINTNEXTLINE(runtime/string): overridden in main() via el parametro results_dir
std::string RESULTADOS = "resultados";

struct Metricas
{
  std::string tramo, planeador, motivo{"sin evaluar"};
  int intento{0};
  bool valida{false};
  double tiempo_plan{0.0}, longitud{0.0}, duracion{0.0}, suavidad{0.0};
  std::size_t puntos{0};
  std::vector<double> movimiento_joint;
};

struct Candidato
{
  MoveGroup::Plan plan;
  Metricas m;
};

struct Perfil
{
  std::string nombre, motivo;
  MoveGroup::Plan plan;
  bool valido{false};
  double duracion{0.0}, suavidad{std::numeric_limits<double>::infinity()};
  double max_vel_art{0.0}, max_acc_art{0.0};
  double max_vel_tcp{0.0}, max_acc_tcp{0.0};
};

geometry_msgs::msg::PoseStamped crear_pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "base_link";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.x = 0.7071067812;
  p.pose.orientation.y = -0.7071067812;
  p.pose.orientation.z = 0.0;
  p.pose.orientation.w = 0.0;
  return p;
}

moveit_msgs::msg::CollisionObject crear_pieza()
{
  moveit_msgs::msg::CollisionObject o;
  o.header.frame_id = "base_link";
  o.id = "workpiece";
  shape_msgs::msg::SolidPrimitive caja;
  caja.type = shape_msgs::msg::SolidPrimitive::BOX;
  caja.dimensions = {0.05, 0.05, 0.07};
  geometry_msgs::msg::Pose p;
  p.orientation.w = 1.0;
  p.position.x = 0.65;
  p.position.y = -0.30;
  p.position.z = 0.040;
  o.primitives.push_back(caja);
  o.primitive_poses.push_back(p);
  o.operation = moveit_msgs::msg::CollisionObject::ADD;
  return o;
}

double segundos(const builtin_interfaces::msg::Duration & d)
{
  return d.sec + 1e-9 * d.nanosec;
}

void calcular_metricas(const trajectory_msgs::msg::JointTrajectory & t, Metricas & m)
{
  m.puntos = t.points.size();
  m.movimiento_joint.assign(t.joint_names.size(), 0.0);
  if (t.points.empty()) return;
  m.duracion = segundos(t.points.back().time_from_start);

  for (std::size_t i = 1; i < t.points.size(); ++i) {
    double norma2 = 0.0;
    const auto & a = t.points[i - 1];
    const auto & b = t.points[i];
    for (std::size_t j = 0; j < std::min(a.positions.size(), b.positions.size()); ++j) {
      const double dq = std::abs(b.positions[j] - a.positions[j]);
      m.movimiento_joint[j] += dq;
      norma2 += dq * dq;
    }
    m.longitud += std::sqrt(norma2);
  }

  // Indicador de suavidad usado en la comparación: variación total de
  // aceleración. Menor valor equivale a mayor suavidad. Se incluye el cambio
  // desde/hacia reposo en los extremos.
  if (!t.points.front().accelerations.empty()) {
    for (double a : t.points.front().accelerations) m.suavidad += std::abs(a);
  }
  for (std::size_t i = 1; i < t.points.size(); ++i) {
    const auto & a = t.points[i - 1].accelerations;
    const auto & b = t.points[i].accelerations;
    for (std::size_t j = 0; j < std::min(a.size(), b.size()); ++j) {
      m.suavidad += std::abs(b[j] - a[j]);
    }
  }
  if (!t.points.back().accelerations.empty()) {
    for (double a : t.points.back().accelerations) m.suavidad += std::abs(a);
  }
}

bool datos_validos(const trajectory_msgs::msg::JointTrajectory & t)
{
  if (t.joint_names.empty() || t.points.size() < 2) return false;
  double anterior = -1.0;
  for (const auto & p : t.points) {
    if (p.positions.size() != t.joint_names.size() ||
      p.velocities.size() != t.joint_names.size() ||
      p.accelerations.size() != t.joint_names.size())
    {
      return false;
    }
    for (double x : p.positions) if (!std::isfinite(x)) return false;
    for (double x : p.velocities) if (!std::isfinite(x)) return false;
    for (double x : p.accelerations) if (!std::isfinite(x)) return false;
    const double tiempo = segundos(p.time_from_start);
    if (!std::isfinite(tiempo) || tiempo < anterior) return false;
    anterior = tiempo;
  }
  return true;
}

bool validar(
  MoveGroup & grupo, const MoveGroup::Plan & plan,
  const geometry_msgs::msg::PoseStamped & meta, std::string & motivo)
{
  const auto & t = plan.trajectory.joint_trajectory;
  if (!datos_validos(t)) {
    motivo = "vacia, incompleta o no finita";
    return false;
  }
  const auto modelo = grupo.getRobotModel();
  const auto * jmg = modelo->getJointModelGroup("ur_manipulator");
  if (!jmg) {
    motivo = "grupo ur_manipulator inexistente";
    return false;
  }
  moveit::core::RobotState estado(modelo);
  estado.setToDefaultValues();
  for (const auto & p : t.points) {
    estado.setVariablePositions(t.joint_names, p.positions);
    estado.update();
    if (!estado.satisfiesBounds(jmg, 1e-6)) {
      motivo = "viola limites articulares";
      return false;
    }
    // La parametrizacion de MoveIt debe respetar velocidad y aceleracion de cada joint.
    for (std::size_t j = 0; j < t.joint_names.size(); ++j) {
      const auto & limites = modelo->getVariableBounds(t.joint_names[j]);
      if (j < p.velocities.size() && limites.velocity_bounded_ &&
        (p.velocities[j] < limites.min_velocity_ - 1e-6 ||
        p.velocities[j] > limites.max_velocity_ + 1e-6))
      {
        motivo = "viola limite de velocidad articular";
        return false;
      }
      if (j < p.accelerations.size() && limites.acceleration_bounded_ &&
        (p.accelerations[j] < limites.min_acceleration_ - 1e-6 ||
        p.accelerations[j] > limites.max_acceleration_ + 1e-6))
      {
        motivo = "viola limite de aceleracion articular";
        return false;
      }
    }
  }
  estado.setVariablePositions(t.joint_names, t.points.back().positions);
  estado.update();
  const auto & tf = estado.getGlobalLinkTransform("tool0");
  const Eigen::Vector3d objetivo(meta.pose.position.x, meta.pose.position.y, meta.pose.position.z);
  const double ep = (tf.translation() - objetivo).norm();
  Eigen::Quaterniond qa(tf.linear());
  Eigen::Quaterniond qm(meta.pose.orientation.w, meta.pose.orientation.x,
    meta.pose.orientation.y, meta.pose.orientation.z);
  qa.normalize(); qm.normalize();
  const double eo = 2.0 * std::acos(std::clamp(std::abs(qa.dot(qm)), 0.0, 1.0));
  if (ep > TOL_POS || eo > TOL_ORI) {
    std::ostringstream s;
    s << "no alcanza meta: error_pos=" << ep << " error_ori=" << eo;
    motivo = s.str();
    return false;
  }
  // Una solucion OMPL exitosa ya fue comprobada contra la PlanningScene por MoveIt.
  motivo = "valida: limites, meta y colisiones comprobados";
  return true;
}

std::string vector_csv(const std::vector<double> & v)
{
  std::ostringstream s;
  s << '"' << std::fixed << std::setprecision(6);
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) s << ';';
    s << v[i];
  }
  return s.str() + '"';
}

void guardar_candidatos(const std::string & tramo, const std::vector<Candidato> & candidatos)
{
  std::filesystem::create_directories(RESULTADOS);
  std::ofstream f(std::string(RESULTADOS) + "/trayectorias_" + tramo + ".csv");
  f << "tramo,planeador,intento,valida,motivo,tiempo_planificacion_s,longitud_articular_rad,"
       "puntos,duracion_s,suavidad,movimiento_por_articulacion_rad\n";
  f << std::fixed << std::setprecision(8);
  for (const auto & c : candidatos) {
    const auto & m = c.m;
    f << m.tramo << ',' << m.planeador << ',' << m.intento << ',' << (m.valida ? "si" : "no")
      << ",\"" << m.motivo << "\"," << m.tiempo_plan << ',' << m.longitud << ',' << m.puntos
      << ',' << m.duracion << ',' << m.suavidad << ',' << vector_csv(m.movimiento_joint) << '\n';
  }
}

bool mejor_que(const Candidato & a, const Candidato & b)
{
  return std::tie(a.m.longitud, a.m.puntos, a.m.duracion, a.m.tiempo_plan, a.m.suavidad) <
         std::tie(b.m.longitud, b.m.puntos, b.m.duracion, b.m.tiempo_plan, b.m.suavidad);
}

bool calcular_estado_aproximacion(
  MoveGroup & grupo, rclcpp::Logger log,
  const geometry_msgs::msg::PoseStamped & contacto,
  const geometry_msgs::msg::PoseStamped & aproximacion,
  std::vector<double> & articulaciones,
  std::vector<double> * articulaciones_contacto = nullptr,
  bool semilla_aleatoria = false)
{
  auto estado = grupo.getCurrentState(3.0);
  const auto * jmg = grupo.getRobotModel()->getJointModelGroup("ur_manipulator");
  if (!estado || !jmg) {
    RCLCPP_ERROR(log, "No se pudo obtener el estado del grupo ur_manipulator");
    return false;
  }
  // El IK numerico converge siempre a la misma rama si parte del mismo
  // estado semilla. Para explorar ramas alternativas (cuando la primera no
  // sostiene un descenso cartesiano continuo, ver main()), se puede forzar
  // una semilla articular aleatoria dentro de los limites del robot.
  if (semilla_aleatoria) estado->setToRandomPositions(jmg);

  // El solver numerico de KDL puede converger a una rama "envuelta" (p.ej.
  // shoulder_pan fuera de [-pi,pi]) que resuelve la pose exacta pero deja el
  // brazo mal condicionado para el descenso cartesiano posterior. canonizar()
  // reescribe cada articulacion de rango >= 2*pi a su representante en
  // [-pi,pi], sin cambiar la pose alcanzada.
  auto canonizar = [&](moveit::core::RobotState & st) {
    std::vector<double> valores;
    st.copyJointGroupPositions(jmg, valores);
    const auto & nombres = jmg->getVariableNames();
    for (size_t i = 0; i < valores.size(); ++i) {
      const auto & limites = st.getRobotModel()->getVariableBounds(nombres[i]);
      if (limites.max_position_ - limites.min_position_ >= 2 * M_PI) {
        const double envuelto = std::remainder(valores[i], 2 * M_PI);
        if (envuelto >= limites.min_position_ && envuelto <= limites.max_position_) {
          valores[i] = envuelto;
        }
      }
    }
    st.setJointGroupPositions(jmg, valores);
  };

  Eigen::Isometry3d contacto_eigen, aproximacion_eigen;
  tf2::fromMsg(contacto.pose, contacto_eigen);
  tf2::fromMsg(aproximacion.pose, aproximacion_eigen);
  if (!estado->setFromIK(jmg, contacto_eigen, "tool0", 2.0)) {
    RCLCPP_ERROR(log, "No existe una rama IK continua entre aproximacion y contacto");
    return false;
  }
  canonizar(*estado);
  if (articulaciones_contacto != nullptr) estado->copyJointGroupPositions(jmg, *articulaciones_contacto);

  // La aproximacion se resuelve cerca de la rama ya canonizada del contacto:
  // pre_pick y pick estan a 0.33 m en linea recta, así que deben quedar en la
  // misma rama IK para que el descenso cartesiano sea continuo. Se prueba con
  // cotas crecientes y, si ninguna cierra, se acepta cualquier rama valida.
  bool resuelto = false;
  for (const double limite : {1.0, 2.0, 3.0}) {
    const std::vector<double> limite_aproximacion(jmg->getVariableCount(), limite);
    if (estado->setFromIK(jmg, aproximacion_eigen, "tool0", limite_aproximacion, 2.0)) {
      resuelto = true;
      break;
    }
  }
  if (!resuelto && !estado->setFromIK(jmg, aproximacion_eigen, "tool0", 2.0)) {
    RCLCPP_ERROR(log, "No existe una rama IK continua entre aproximacion y contacto");
    return false;
  }
  canonizar(*estado);
  estado->copyJointGroupPositions(jmg, articulaciones);
  return true;
}

// Barra de progreso en vivo en la misma terminal: se reescribe en el sitio
// (retorno de carro) mientras se calculan los 20 intentos, y se marca con
// [OK]/[X] apenas termina cada uno.
void barra_progreso(int hecho, int total, const std::string & planeador, int intento, bool valida)
{
  constexpr int ANCHO = 20;
  const int llenos = static_cast<int>(std::round(ANCHO * static_cast<double>(hecho) / total));
  std::string barra(llenos, '#');
  barra.append(ANCHO - llenos, '-');
  std::fprintf(
    stderr, "\r[%s] intento %d/%d (%s #%02d) %s     ", barra.c_str(), hecho, total,
    planeador.c_str(), intento, valida ? "[OK]" : "[X]");
  std::fflush(stderr);
  if (hecho == total) std::fprintf(stderr, "\n");
}

// El current_state_monitor de MoveGroupInterface corre en el hilo del
// executor (aparte del hilo principal) y procesa /joint_states de forma
// asincrona. execute() retorna en cuanto el controlador reporta la
// trayectoria terminada, pero eso no garantiza que el monitor ya haya
// procesado el ultimo mensaje de estado: getCurrentState()/getCurrentPose()
// pueden devolver todavia la pose anterior al movimiento. setStartStateTo-
// CurrentState() no ayuda porque lee del mismo cache. Por eso, tras cada
// execute(), se espera activamente (con timeout) a que el estado leido
// coincida con las articulaciones finales del plan antes de continuar.
bool esperar_estado_sincronizado(
  MoveGroup & grupo, const std::vector<double> & meta_articular, double timeout_s = 3.0)
{
  const auto * jmg = grupo.getRobotModel()->getJointModelGroup("ur_manipulator");
  const auto inicio = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(
           std::chrono::steady_clock::now() - inicio).count() < timeout_s)
  {
    auto estado = grupo.getCurrentState(0.2);
    if (estado && jmg) {
      std::vector<double> actuales;
      estado->copyJointGroupPositions(jmg, actuales);
      if (actuales.size() == meta_articular.size()) {
        double error = 0.0;
        for (size_t i = 0; i < actuales.size(); ++i) {
          error = std::max(error, std::abs(actuales[i] - meta_articular[i]));
        }
        if (error < 1e-5) return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

bool planificar_y_ejecutar_mejor(
  MoveGroup & grupo, rclcpp::Logger log, const std::string & tramo,
  const geometry_msgs::msg::PoseStamped & meta,
  const std::vector<double> & articulaciones_meta)
{
  const std::array<std::string, 2> planeadores =
    {"RRTConnectkConfigDefault", "RRTstarkConfigDefault"};
  std::vector<Candidato> candidatos;
  grupo.clearPoseTargets();
  if (!grupo.setJointValueTarget(articulaciones_meta)) return false;

  // Los veinte planes se calculan antes de ejecutar; todos parten del mismo estado real.
  const int total_intentos = static_cast<int>(planeadores.size()) * INTENTOS;
  for (const auto & planner : planeadores) {
    grupo.setPlannerId(planner);
    for (int intento = 1; intento <= INTENTOS; ++intento) {
      Candidato c;
      c.m = {tramo, planner, "OMPL no encontro solucion", intento};
      grupo.setStartStateToCurrentState();
      const auto inicio = std::chrono::steady_clock::now();
      const bool encontrado = static_cast<bool>(grupo.plan(c.plan));
      c.m.tiempo_plan = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - inicio).count();
      if (encontrado) {
        calcular_metricas(c.plan.trajectory.joint_trajectory, c.m);
        c.m.valida = validar(grupo, c.plan, meta, c.m.motivo);
      }
      barra_progreso(
        static_cast<int>(candidatos.size()) + 1, total_intentos, planner, intento, c.m.valida);
      candidatos.push_back(std::move(c));
    }
  }
  grupo.clearPoseTargets();
  guardar_candidatos(tramo, candidatos);

  const Candidato * mejor = nullptr;
  int cumplieron = 0;
  RCLCPP_INFO(log, "%s: checklist de intentos (%zu en total)", tramo.c_str(), candidatos.size());
  for (const auto & c : candidatos) {
    if (c.m.valida) {
      ++cumplieron;
      RCLCPP_INFO(
        log, "  [✓] %s intento %02d: cumplio (longitud=%.5f rad, %zu puntos, %.3f s)",
        c.m.planeador.c_str(), c.m.intento, c.m.longitud, c.m.puntos, c.m.duracion);
      if (!mejor || mejor_que(c, *mejor)) mejor = &c;
    } else {
      RCLCPP_INFO(
        log, "  [✗] %s intento %02d: no cumplio (%s)",
        c.m.planeador.c_str(), c.m.intento, c.m.motivo.c_str());
    }
  }
  RCLCPP_INFO(
    log, "%s: %d de %zu intentos cumplieron", tramo.c_str(), cumplieron, candidatos.size());
  if (!mejor) {
    RCLCPP_ERROR(log, "%s: no hay candidato valido; no se mueve el robot", tramo.c_str());
    return false;
  }
  RCLCPP_INFO(
    log, "%s: gana %s intento %d, longitud %.5f rad; se ejecuta solo este, al final",
    tramo.c_str(), mejor->m.planeador.c_str(), mejor->m.intento, mejor->m.longitud);
  if (!static_cast<bool>(grupo.execute(mejor->plan))) return false;
  if (!esperar_estado_sincronizado(grupo, articulaciones_meta)) {
    RCLCPP_WARN(
      log, "%s: el estado leido tras execute() no convergio a la meta dentro del timeout",
      tramo.c_str());
  }
  return true;
}

double ley(double tau, bool quintico)
{
  return quintico ? 10 * std::pow(tau, 3) - 15 * std::pow(tau, 4) + 6 * std::pow(tau, 5) :
         3 * tau * tau - 2 * tau * tau * tau;
}

double ley_inversa(double s, bool quintico)
{
  double a = 0.0, b = 1.0;
  for (int i = 0; i < 50; ++i) {
    const double m = (a + b) / 2.0;
    if (ley(m, quintico) < s) a = m; else b = m;
  }
  return (a + b) / 2.0;
}

void derivadas(trajectory_msgs::msg::JointTrajectory & t)
{
  const std::size_t n = t.joint_names.size();
  for (auto & p : t.points) {
    p.velocities.assign(n, 0.0);
    p.accelerations.assign(n, 0.0);
  }
  if (t.points.size() < 2) return;

  // Velocidad: cero en los extremos por condición de reposo; diferencias
  // centradas en los puntos interiores.
  for (std::size_t i = 1; i + 1 < t.points.size(); ++i) {
    const double dt = segundos(t.points[i + 1].time_from_start) -
      segundos(t.points[i - 1].time_from_start);
    if (dt <= 1e-9) continue;
    for (std::size_t j = 0; j < n; ++j) {
      t.points[i].velocities[j] =
        (t.points[i + 1].positions[j] - t.points[i - 1].positions[j]) / dt;
    }
  }

  // Aceleración interior mediante diferencias centradas.
  for (std::size_t i = 1; i + 1 < t.points.size(); ++i) {
    const double dt = segundos(t.points[i + 1].time_from_start) -
      segundos(t.points[i - 1].time_from_start);
    if (dt <= 1e-9) continue;
    for (std::size_t j = 0; j < n; ++j) {
      t.points[i].accelerations[j] =
        (t.points[i + 1].velocities[j] - t.points[i - 1].velocities[j]) / dt;
    }
  }

  // Aceleración en los extremos con diferencia unilateral. Esto permite que
  // el cúbico refleje su aceleración no nula en los bordes, mientras el
  // quíntico tiende a cero.
  const double dt0 = segundos(t.points[1].time_from_start) -
    segundos(t.points[0].time_from_start);
  if (dt0 > 1e-9) {
    for (std::size_t j = 0; j < n; ++j) {
      t.points[0].accelerations[j] =
        (t.points[1].velocities[j] - t.points[0].velocities[j]) / dt0;
    }
  }
  const std::size_t last = t.points.size() - 1;
  const double dt1 = segundos(t.points[last].time_from_start) -
    segundos(t.points[last - 1].time_from_start);
  if (dt1 > 1e-9) {
    for (std::size_t j = 0; j < n; ++j) {
      t.points[last].accelerations[j] =
        (t.points[last].velocities[j] - t.points[last - 1].velocities[j]) / dt1;
    }
  }
}

bool medir_tcp(MoveGroup & grupo, const trajectory_msgs::msg::JointTrajectory & t, Perfil & perfil)
{
  const auto modelo = grupo.getRobotModel();
  const auto * jmg = modelo->getJointModelGroup("ur_manipulator");
  const auto * tool = modelo->getLinkModel("tool0");
  if (!jmg || !tool || t.points.size() < 2) return false;

  moveit::core::RobotState estado(modelo);
  std::vector<Eigen::Vector3d> velocidades_tcp(t.points.size(), Eigen::Vector3d::Zero());

  for (std::size_t i = 0; i < t.points.size(); ++i) {
    const auto & punto = t.points[i];
    if (punto.velocities.size() != t.joint_names.size()) return false;
    estado.setVariablePositions(t.joint_names, punto.positions);
    estado.update();

    Eigen::MatrixXd jacobiano;
    if (!estado.getJacobian(jmg, tool, Eigen::Vector3d::Zero(), jacobiano, false)) return false;
    if (jacobiano.rows() != 6 || jacobiano.cols() != static_cast<Eigen::Index>(punto.velocities.size())) {
      return false;
    }
    Eigen::VectorXd qdot(punto.velocities.size());
    for (std::size_t j = 0; j < punto.velocities.size(); ++j) qdot(j) = punto.velocities[j];
    const Eigen::VectorXd xdot = jacobiano * qdot;
    velocidades_tcp[i] = xdot.head<3>();
    perfil.max_vel_tcp = std::max(perfil.max_vel_tcp, velocidades_tcp[i].norm());
  }

  // Aceleración lineal aproximada a partir de la velocidad TCP obtenida con J*qdot.
  for (std::size_t i = 1; i < t.points.size(); ++i) {
    const double dt = segundos(t.points[i].time_from_start) - segundos(t.points[i - 1].time_from_start);
    if (dt <= 1e-9) return false;
    const double a = (velocidades_tcp[i] - velocidades_tcp[i - 1]).norm() / dt;
    perfil.max_acc_tcp = std::max(perfil.max_acc_tcp, a);
  }
  return std::isfinite(perfil.max_vel_tcp) && std::isfinite(perfil.max_acc_tcp);
}

Perfil temporizar(
  MoveGroup & grupo, const MoveGroup::Plan & geometria, const std::string & nombre,
  double distancia_referencia, double vmax, double amax)
{
  Perfil r;
  r.nombre = nombre;
  r.plan = geometria;
  auto & t = r.plan.trajectory.joint_trajectory;
  if (t.points.size() < 2 || distancia_referencia <= 0.0) {
    r.motivo = "geometria vacia";
    return r;
  }

  // La ley temporal se aplica a la distancia CARTESIANA recorrida por tool0,
  // no al índice ni a la distancia articular. Así el límite del enunciado es
  // realmente una restricción sobre la velocidad del TCP.
  const auto modelo = grupo.getRobotModel();
  moveit::core::RobotState estado(modelo);

  // computeCartesianPath con un eef_step fino puede repetir dos waypoints
  // casi identicos en el espacio cartesiano (ruido numerico de la IK). Sin
  // filtrarlos, la reparametrizacion por longitud de arco les asigna un
  // intervalo de tiempo casi nulo, y la diferencia finita de velocidad TCP
  // produce una aceleracion artificialmente enorme. Se descartan los puntos
  // cuyo avance cartesiano es despreciable, conservando siempre el primero
  // y el ultimo.
  constexpr double INCREMENTO_MINIMO_M = 1e-5;
  std::vector<Eigen::Vector3d> posiciones(t.points.size());
  for (std::size_t i = 0; i < t.points.size(); ++i) {
    estado.setVariablePositions(t.joint_names, t.points[i].positions);
    estado.update();
    posiciones[i] = estado.getGlobalLinkTransform("tool0").translation();
  }
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> filtrados;
  std::vector<Eigen::Vector3d> posiciones_filtradas;
  for (std::size_t i = 0; i < t.points.size(); ++i) {
    const bool es_extremo = (i == 0 || i == t.points.size() - 1);
    if (es_extremo || (posiciones[i] - posiciones_filtradas.back()).norm() > INCREMENTO_MINIMO_M) {
      filtrados.push_back(t.points[i]);
      posiciones_filtradas.push_back(posiciones[i]);
    }
  }
  t.points = std::move(filtrados);

  std::vector<double> l(t.points.size(), 0.0);
  for (std::size_t i = 1; i < posiciones_filtradas.size(); ++i) {
    l[i] = l[i - 1] + (posiciones_filtradas[i] - posiciones_filtradas[i - 1]).norm();
  }
  if (t.points.size() < 2 || l.back() <= 1e-12) {
    r.motivo = "sin movimiento cartesiano";
    return r;
  }

  const bool q = nombre == "quintico";
  const double factor_v = q ? 1.875 : 1.5;
  const double factor_a = q ? 10.0 / std::sqrt(3.0) : 6.0;
  const double distancia = l.back();
  r.duracion = std::max(factor_v * distancia / vmax, std::sqrt(factor_a * distancia / amax));

  for (std::size_t i = 0; i < t.points.size(); ++i) {
    const double tiempo = r.duracion * ley_inversa(l[i] / l.back(), q);
    t.points[i].time_from_start = rclcpp::Duration::from_seconds(tiempo);
  }
  derivadas(t);

  Metricas m;
  calcular_metricas(t, m);
  r.suavidad = m.suavidad;
  for (const auto & punto : t.points) {
    for (double v : punto.velocities) r.max_vel_art = std::max(r.max_vel_art, std::abs(v));
    for (double a : punto.accelerations) r.max_acc_art = std::max(r.max_acc_art, std::abs(a));
  }

  if (!datos_validos(t) || !medir_tcp(grupo, t, r)) {
    r.motivo = "datos temporales o TCP invalidos";
    return r;
  }

  // Pequeña tolerancia numérica para derivadas discretas; no se permite una
  // violación material de los límites del enunciado.
  constexpr double TOL_VEL_TCP = 1e-3;
  constexpr double TOL_ACC_TCP = 2e-3;
  if (r.max_vel_tcp > vmax + TOL_VEL_TCP) {
    std::ostringstream motivo;
    motivo << "viola velocidad TCP: " << r.max_vel_tcp << " > " << vmax;
    r.motivo = motivo.str();
    return r;
  }
  if (r.max_acc_tcp > amax + TOL_ACC_TCP) {
    std::ostringstream motivo;
    motivo << "viola aceleracion TCP: " << r.max_acc_tcp << " > " << amax;
    r.motivo = motivo.str();
    return r;
  }

  r.valido = true;
  r.motivo = "valido";
  return r;
}

void guardar_perfiles(
  const std::string & tramo, const std::vector<Perfil> & perfiles, double vmax, double amax)
{
  std::filesystem::create_directories(RESULTADOS);
  std::ofstream f(std::string(RESULTADOS) + "/perfiles_" + tramo + ".csv");
  f << "tramo,perfil,valido,duracion_s,variacion_aceleracion_articular,"
       "max_velocidad_articular,max_aceleracion_articular,max_velocidad_tcp_m_s,"
       "max_aceleracion_tcp_m_s2,limite_tcp_velocidad_m_s,limite_tcp_aceleracion_m_s2,motivo\n";
  for (const auto & p : perfiles)
    f << tramo << ',' << p.nombre << ',' << (p.valido ? "si" : "no") << ',' << p.duracion
      << ',' << p.suavidad << ',' << p.max_vel_art << ',' << p.max_acc_art << ','
      << p.max_vel_tcp << ',' << p.max_acc_tcp << ',' << vmax << ',' << amax << ','
      << p.motivo << '\n';
}

void guardar_perfil_seleccionado(const std::string & tramo, const Perfil & perfil)
{
  std::filesystem::create_directories(RESULTADOS);
  std::ofstream f(std::string(RESULTADOS) + "/perfil_seleccionado_" + tramo + ".txt");
  f << "tramo=" << tramo << '\n'
    << "perfil=" << perfil.nombre << '\n'
    << "duracion_s=" << std::setprecision(10) << perfil.duracion << '\n'
    << "variacion_aceleracion_articular=" << perfil.suavidad << '\n'
    << "max_velocidad_articular=" << perfil.max_vel_art << '\n'
    << "max_aceleracion_articular=" << perfil.max_acc_art << '\n'
    << "max_velocidad_tcp_m_s=" << perfil.max_vel_tcp << '\n'
    << "max_aceleracion_tcp_m_s2=" << perfil.max_acc_tcp << '\n'
    << "criterio=menor variacion de aceleracion; desempate menor duracion\n";
}

bool mover_cartesiano(
  MoveGroup & grupo, rclcpp::Logger log, const std::string & tramo,
  const geometry_msgs::msg::PoseStamped & meta, double vmax, double amax,
  const std::string & perfil_forzado, std::string * perfil_elegido,
  bool solo_verificar = false, const moveit::core::RobotState * estado_inicio = nullptr)
{
  // getCurrentPose()/getCurrentState() siempre leen el estado FISICO real del
  // current_state_monitor, sin importar si antes se llamo a setStartState()
  // con un estado forzado (eso solo afecta a plan()/computeCartesianPath()).
  // Si se pasa un estado_inicio explicito (verificacion en seco de una rama,
  // sin mover el robot), la pose de partida debe calcularse por FK de ESE
  // estado, no de la pose fisica real.
  geometry_msgs::msg::PoseStamped actual;
  if (estado_inicio != nullptr) {
    const auto & tf = estado_inicio->getGlobalLinkTransform("tool0");
    actual.header.frame_id = "base_link";
    actual.pose = tf2::toMsg(tf);
  } else {
    actual = grupo.getCurrentPose("tool0");
  }
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // El taller pide entre 3 y 4 waypoints intermedios. Aquí se usan cuatro
  // intermedios más la meta final sobre una línea recta.
  for (int i = 1; i <= 5; ++i) {
    const double s = i / 5.0;
    auto p = actual.pose;
    p.position.x += s * (meta.pose.position.x - actual.pose.position.x);
    p.position.y += s * (meta.pose.position.y - actual.pose.position.y);
    p.position.z += s * (meta.pose.position.z - actual.pose.position.z);
    p.orientation = meta.pose.orientation;
    waypoints.push_back(p);
  }

  if (estado_inicio != nullptr) {
    // Verificacion en seco: se planifica desde el estado forzado por el
    // llamador (buscar_rama_continua), no desde el estado fisico real.
    grupo.setStartState(*estado_inicio);
  } else {
    // computeCartesianPath usa el estado interno que MoveGroupInterface tiene
    // cacheado, que puede no haberse actualizado todavia tras la ejecucion
    // anterior (el current_state_monitor procesa /joint_states de forma
    // asincrona). Sin esto, a veces interpola desde una pose vieja (HOME) en
    // vez de "actual", produciendo una trayectoria mucho mas larga de lo
    // esperado y disparando falsas violaciones de aceleracion TCP.
    grupo.setStartStateToCurrentState();
  }

  const double dx = meta.pose.position.x - actual.pose.position.x;
  const double dy = meta.pose.position.y - actual.pose.position.y;
  const double dz = meta.pose.position.z - actual.pose.position.z;
  const double distancia = std::sqrt(dx * dx + dy * dy + dz * dz);

  // La IK numerica interna de computeCartesianPath no siempre converge de
  // forma continua en este descenso (orientacion fija, tramo casi vertical):
  // unas veces se detiene antes de completar la linea (autocolision o sin
  // solucion en un paso intermedio) y otras la completa pero con un salto
  // puntual entre dos pasos consecutivos de 2 mm, que se traduce en un pico
  // de aceleracion del TCP muy por encima de lo fisicamente esperado para
  // esa distancia. Igual que en 4A (donde se generan 20 planes OMPL y se
  // valida cada uno), aqui se repite el calculo geometrico varias veces y
  // solo se acepta una geometria cuya reparametrizacion cubica o quintica
  // cumpla de verdad las restricciones del enunciado.
  constexpr int INTENTOS_CARTESIANOS = 15;
  std::vector<Perfil> todos;
  int completos = 0;
  for (int intento = 1; intento <= INTENTOS_CARTESIANOS; ++intento) {
    moveit_msgs::msg::RobotTrajectory msg;
    // Paso fino (2 mm) para que la IK numerica de cada tramo permanezca cerca
    // de la solucion anterior y no pierda continuidad de rama cerca de PICK/PLACE.
    const double fraccion = grupo.computeCartesianPath(waypoints, 0.002, msg, true);
    if (fraccion < 0.999) {
      RCLCPP_INFO(
        log, "%s: intento %d/%d de geometria cartesiana alcanzo %.1f%% (descartado)",
        tramo.c_str(), intento, INTENTOS_CARTESIANOS, fraccion * 100.0);
      continue;
    }
    ++completos;

    MoveGroup::Plan geometria;
    geometria.trajectory = msg;
    // La misma geometría cartesiana se reparametriza temporalmente con las
    // dos leyes. La geometría y el perfil temporal son conceptos distintos.
    std::array<Perfil, 2> perfiles = {
      temporizar(grupo, geometria, "cubico", distancia, vmax, amax),
      temporizar(grupo, geometria, "quintico", distancia, vmax, amax)};

    // computeCartesianPath comprobó colisiones. validar() comprueba además
    // meta, límites articulares, velocidad, aceleración y valores finitos.
    bool alguno_valido = false;
    for (auto & p : perfiles) {
      if (p.valido) p.valido = validar(grupo, p.plan, meta, p.motivo);
      RCLCPP_INFO(
        log, "%s: intento %d/%d %s: T=%.3f s suavidad=%.6f vmax_tcp=%.4f amax_tcp=%.4f, %s",
        tramo.c_str(), intento, INTENTOS_CARTESIANOS, p.nombre.c_str(), p.duracion, p.suavidad,
        p.max_vel_tcp, p.max_acc_tcp, p.motivo.c_str());
      if (p.valido && (perfil_forzado.empty() || p.nombre == perfil_forzado)) alguno_valido = true;
      todos.push_back(std::move(p));
    }
    if (alguno_valido) break;  // geometria valida encontrada; no hace falta seguir probando
  }

  RCLCPP_INFO(
    log, "%s: %d de %d intentos de geometria cartesiana completaron la linea al 100%%",
    tramo.c_str(), completos, INTENTOS_CARTESIANOS);
  if (!solo_verificar) guardar_perfiles(tramo, todos, vmax, amax);

  const Perfil * mejor = nullptr;
  for (const auto & p : todos) {
    if (!p.valido) continue;
    if (!perfil_forzado.empty()) {
      if (p.nombre == perfil_forzado) mejor = &p;
      continue;
    }
    // El indicador guardado es variación acumulada de aceleración: menor
    // valor equivale a mayor suavidad. En empate se prefiere menor duración.
    if (!mejor || std::tie(p.suavidad, p.duracion) <
      std::tie(mejor->suavidad, mejor->duracion))
    {
      mejor = &p;
    }
  }

  if (!mejor) {
    if (perfil_forzado.empty()) {
      RCLCPP_ERROR(
        log, "%s: ningun perfil valido en %d intentos de geometria cartesiana",
        tramo.c_str(), INTENTOS_CARTESIANOS);
    } else {
      RCLCPP_ERROR(
        log, "%s: el perfil seleccionado en 4B (%s) no es valido en este tramo",
        tramo.c_str(), perfil_forzado.c_str());
    }
    return false;
  }

  if (perfil_elegido != nullptr) *perfil_elegido = mejor->nombre;
  if (solo_verificar) {
    RCLCPP_INFO(
      log, "%s: rama verificada en seco, perfil %s es fisicamente valido (no se ejecuta)",
      tramo.c_str(), mejor->nombre.c_str());
    return true;
  }
  guardar_perfil_seleccionado(tramo, *mejor);
  RCLCPP_INFO(
    log, "%s: se ejecuta solo el perfil %s", tramo.c_str(), mejor->nombre.c_str());
  if (!static_cast<bool>(grupo.execute(mejor->plan))) return false;
  if (!mejor->plan.trajectory.joint_trajectory.points.empty()) {
    if (!esperar_estado_sincronizado(
          grupo, mejor->plan.trajectory.joint_trajectory.points.back().positions))
    {
      RCLCPP_WARN(
        log, "%s: el estado leido tras execute() no convergio a la meta dentro del timeout",
        tramo.c_str());
    }
  }
  return true;
}

bool mover_home(MoveGroup & grupo, rclcpp::Logger log)
{
  grupo.clearPoseTargets();
  if (!grupo.setNamedTarget("HOME")) return false;
  grupo.setPlannerId("RRTConnectkConfigDefault");
  grupo.setStartStateToCurrentState();
  MoveGroup::Plan plan;
  if (!static_cast<bool>(grupo.plan(plan))) {
    RCLCPP_ERROR(log, "No se pudo planificar HOME");
    return false;
  }
  if (!static_cast<bool>(grupo.execute(plan))) return false;
  if (!plan.trajectory.joint_trajectory.points.empty()) {
    esperar_estado_sincronizado(grupo, plan.trajectory.joint_trajectory.points.back().positions);
  }
  return true;
}

// Salto directo a una configuracion articular (p.ej. PICK), sin pasar por los
// tramos intermedios del ciclo completo. Usado por el modo "solo_pick_place".
bool mover_directo(
  MoveGroup & grupo, rclcpp::Logger log, const std::vector<double> & articulaciones,
  const char * nombre)
{
  grupo.clearPoseTargets();
  if (!grupo.setJointValueTarget(articulaciones)) return false;
  grupo.setPlannerId("RRTConnectkConfigDefault");
  grupo.setStartStateToCurrentState();
  MoveGroup::Plan plan;
  if (!static_cast<bool>(grupo.plan(plan))) {
    RCLCPP_ERROR(log, "No se pudo planificar el salto directo a %s", nombre);
    return false;
  }
  if (!static_cast<bool>(grupo.execute(plan))) return false;
  esperar_estado_sincronizado(grupo, articulaciones);
  return true;
}

// calcular_estado_aproximacion() sólo exige que la rama IK sea continua
// entre "aproximacion" y "contacto" (limites articulares, sin envolvimiento).
// Eso no garantiza que el descenso cartesiano posterior (computeCartesianPath,
// paso a paso) sea fisicamente suave: segun la rama, puede quedar cerca de
// una configuracion mal condicionada y producir saltos articulares puntuales
// que disparan la aceleracion del TCP muy por encima del limite del taller.
// Aqui se prueban varias ramas (primero la semilla natural, luego semillas
// aleatorias) y, para cada una, se verifica en seco -sin mover el robot
// fisico, con un estado de partida forzado- que el tramo cartesiano completo
// cumple v <= vmax y a <= amax. Solo se acepta una rama que ya pasó esa
// prueba antes de comprometerse a ejecutar el movimiento articular real.
bool buscar_rama_continua(
  MoveGroup & grupo, rclcpp::Logger log, const std::string & tramo,
  const geometry_msgs::msg::PoseStamped & contacto,
  const geometry_msgs::msg::PoseStamped & aproximacion,
  double vmax, double amax, std::vector<double> & articulaciones)
{
  constexpr int INTENTOS_RAMA = 6;
  const auto * jmg = grupo.getRobotModel()->getJointModelGroup("ur_manipulator");
  bool encontrada = false;
  for (int intento = 1; intento <= INTENTOS_RAMA && !encontrada; ++intento) {
    std::vector<double> candidata;
    if (!calcular_estado_aproximacion(
          grupo, log, contacto, aproximacion, candidata, nullptr, intento > 1))
    {
      continue;
    }

    auto estado_prueba = grupo.getCurrentState(3.0);
    if (!estado_prueba || !jmg) continue;
    estado_prueba->setJointGroupPositions(jmg, candidata);
    estado_prueba->update();

    std::string perfil_dummy;
    const bool geom_ok = mover_cartesiano(
      grupo, log, tramo + "_verificacion_rama", contacto, vmax, amax, "", &perfil_dummy, true,
      estado_prueba.get());
    grupo.setStartStateToCurrentState();

    RCLCPP_INFO(
      log, "%s: verificacion de rama %d/%d %s", tramo.c_str(), intento, INTENTOS_RAMA,
      geom_ok ? "OK, se usa esta rama" : "no sostiene un descenso cartesiano suave");
    if (geom_ok) {
      articulaciones = candidata;
      encontrada = true;
    }
  }
  if (!encontrada) {
    RCLCPP_ERROR(
      log, "%s: ninguna de %d ramas IK sostiene un descenso cartesiano valido",
      tramo.c_str(), INTENTOS_RAMA);
  }
  return encontrada;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nodo = std::make_shared<rclcpp::Node>(
    "ur5_pick_place_sequence",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const std::string kinematics = "robot_description_kinematics.ur_manipulator.";
  if (!nodo->has_parameter(kinematics + "kinematics_solver"))
    nodo->declare_parameter<std::string>(
      kinematics + "kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  if (!nodo->has_parameter(kinematics + "kinematics_solver_search_resolution"))
    nodo->declare_parameter<double>(kinematics + "kinematics_solver_search_resolution", 0.005);
  if (!nodo->has_parameter(kinematics + "kinematics_solver_timeout"))
    nodo->declare_parameter<double>(kinematics + "kinematics_solver_timeout", 0.05);
  if (!nodo->has_parameter("hasta_tramo"))
    nodo->declare_parameter<std::string>("hasta_tramo", "FULL");
  if (!nodo->has_parameter("results_dir"))
    nodo->declare_parameter<std::string>("results_dir", RESULTADOS);
  RESULTADOS = nodo->get_parameter("results_dir").as_string();
  if (!nodo->has_parameter("solo_pick_place"))
    nodo->declare_parameter<bool>("solo_pick_place", false);
  const bool solo_pick_place = nodo->get_parameter("solo_pick_place").as_bool();

  const std::string hasta_tramo = nodo->get_parameter("hasta_tramo").as_string();
  const std::array<std::string, 5> modos = {"4A", "4B", "4C", "4D", "FULL"};
  if (std::find(modos.begin(), modos.end(), hasta_tramo) == modos.end()) {
    RCLCPP_ERROR(nodo->get_logger(), "hasta_tramo debe ser 4A, 4B, 4C, 4D o FULL");
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nodo);
  std::thread hilo([&executor]() {executor.spin();});

  MoveGroup grupo(nodo, "ur_manipulator");
  grupo.setPoseReferenceFrame("base_link");
  grupo.setEndEffectorLink("tool0");
  grupo.setPlanningTime(8.0);
  grupo.setNumPlanningAttempts(1);  // Cada intento externo produce una fila del CSV.
  grupo.setMaxVelocityScalingFactor(0.20);
  grupo.setMaxAccelerationScalingFactor(0.20);
  grupo.allowReplanning(false);

  // El current_state_monitor recien creado no tiene ningun /joint_states
  // procesado todavia. Si se planifica antes de que llegue el primero, el
  // estado de partida queda en los valores por defecto del modelo (no la
  // pose fisica real), y el controlador aborta la ejecucion de inmediato
  // porque el primer punto de la trayectoria no coincide con el estado real.
  // getCurrentState(timeout) espera activamente a que llegue un estado
  // completo antes de continuar.
  if (!grupo.getCurrentState(5.0)) {
    RCLCPP_ERROR(
      nodo->get_logger(), "No se recibio el estado inicial del robot (/joint_states)");
    executor.cancel();
    hilo.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::PlanningSceneInterface escena;
  const auto objetos = escena.getKnownObjectNames();
  const bool escena_base_lista =
    std::find(objetos.begin(), objetos.end(), "pick_surface") != objetos.end() &&
    std::find(objetos.begin(), objetos.end(), "place_surface") != objetos.end() &&
    std::find(objetos.begin(), objetos.end(), "obstacle") != objetos.end();
  if (!escena_base_lista) {
    RCLCPP_ERROR(
      nodo->get_logger(),
      "Faltan objetos de la escena. Ejecute primero: ros2 run ur5_pick_place planning_scene_setup");
    executor.cancel();
    hilo.join();
    rclcpp::shutdown();
    return 1;
  }

  // Se reconstruye sólo la pieza para garantizar una posición inicial conocida.
  grupo.detachObject("workpiece");
  escena.removeCollisionObjects({"workpiece"});
  rclcpp::sleep_for(std::chrono::milliseconds(400));
  bool exito = escena.applyCollisionObject(crear_pieza());

  const auto pre_pick = crear_pose(0.65, -0.30, 0.45);
  const auto pick = crear_pose(0.65, -0.30, 0.12);
  const auto pre_place = crear_pose(0.65, 0.30, 0.45);
  const auto place = crear_pose(0.65, 0.30, 0.12);

  if (solo_pick_place) {
    // Modo rapido de demostracion: salta directo a PICK (sin animar HOME/4A/4B),
    // adjunta la pieza y compara 20 caminos libres directo hasta PLACE.
    std::vector<double> q_pick, q_place;
    exito = exito && calcular_estado_aproximacion(
      grupo, nodo->get_logger(), pick, place, q_place, &q_pick);
    exito = exito && mover_directo(grupo, nodo->get_logger(), q_pick, "PICK");
    if (exito) exito = grupo.attachObject(
      "workpiece", "tool0", {"tool0", "flange", "wrist_3_link"});
    if (exito) RCLCPP_INFO(nodo->get_logger(), "solo_pick_place: en PICK, pieza adjuntada");
    exito = exito && planificar_y_ejecutar_mejor(
      grupo, nodo->get_logger(), "pick_a_place_directo", place, q_place);
    if (exito) exito = grupo.detachObject("workpiece");
    if (exito) {
      RCLCPP_INFO(nodo->get_logger(), "solo_pick_place: pieza liberada en PLACE");
    } else {
      RCLCPP_ERROR(nodo->get_logger(), "solo_pick_place: ciclo detenido de forma segura");
    }
    executor.cancel();
    hilo.join();
    rclcpp::shutdown();
    return exito ? 0 : 1;
  }

  // Primero se lleva el robot a HOME: así la semilla de calcular_estado_aproximacion()
  // es siempre el mismo estado conocido y la rama IK de q_pre_pick/q_pick es reproducible
  // entre ejecuciones (si se calculara antes, la semilla sería la pose física arbitraria
  // en la que haya quedado el robot al iniciar el proceso).
  exito = exito && mover_home(grupo, nodo->get_logger());

  std::vector<double> q_pre_pick;
  exito = exito && buscar_rama_continua(
    grupo, nodo->get_logger(), "4B", pick, pre_pick, 0.200, 0.300, q_pre_pick);

  // 4A: primero se llevan todos los experimentos a la misma condición HOME.
  exito = exito && planificar_y_ejecutar_mejor(
    grupo, nodo->get_logger(), "4A_home_pre_pick", pre_pick, q_pre_pick);
  if (exito && hasta_tramo == "4A") {
    RCLCPP_INFO(nodo->get_logger(), "Demostracion detenida despues de 4A por solicitud");
  }

  std::string perfil_4b;
  if (exito && hasta_tramo != "4A") {
    // 4B: comparar cúbico/quintico y recordar el elegido para reutilizarlo en 4D.
    exito = mover_cartesiano(
      grupo, nodo->get_logger(), "4B_pre_pick_pick", pick, 0.200, 0.300, "", &perfil_4b);
    if (exito) exito = grupo.attachObject(
      "workpiece", "tool0", {"tool0", "flange", "wrist_3_link"});
    if (exito) {
      RCLCPP_INFO(
        nodo->get_logger(), "4B: pieza adjuntada; perfil seleccionado=%s", perfil_4b.c_str());
    }
  }

  if (exito && hasta_tramo == "4B") {
    RCLCPP_INFO(nodo->get_logger(), "Demostracion detenida despues de 4B por solicitud");
  }

  if (exito && hasta_tramo != "4A" && hasta_tramo != "4B") {
    // 4C parte realmente desde PICK y mantiene la pieza adjunta. No se inserta
    // un retiro cartesiano extra, para respetar la definición del taller.
    std::vector<double> q_pre_place;
    exito = buscar_rama_continua(
      grupo, nodo->get_logger(), "4D", place, pre_place, 0.100, 0.020, q_pre_place);
    exito = exito && planificar_y_ejecutar_mejor(
      grupo, nodo->get_logger(), "4C_pick_pre_place", pre_place, q_pre_place);
  }

  if (exito && hasta_tramo == "4C") {
    RCLCPP_INFO(nodo->get_logger(), "Demostracion detenida despues de 4C por solicitud");
  }

  if (exito && (hasta_tramo == "4D" || hasta_tramo == "FULL")) {
    // 4D usa exactamente el perfil que ganó en 4B. Si ese perfil no cumple
    // los límites en 4D, el ciclo se detiene; no se cambia silenciosamente.
    exito = mover_cartesiano(
      grupo, nodo->get_logger(), "4D_pre_place_place", place, 0.100, 0.020,
      perfil_4b, nullptr);
    if (exito) exito = grupo.detachObject("workpiece");
    if (exito) RCLCPP_INFO(nodo->get_logger(), "4D: pieza liberada despues de alcanzar PLACE");
  }

  if (exito && hasta_tramo == "FULL") {
    RCLCPP_INFO(nodo->get_logger(), "Ciclo pick-and-place completado");
  } else if (!exito) {
    RCLCPP_ERROR(nodo->get_logger(), "Ciclo detenido de forma segura");
  }

  executor.cancel();
  hilo.join();
  rclcpp::shutdown();
  return exito ? 0 : 1;
}
