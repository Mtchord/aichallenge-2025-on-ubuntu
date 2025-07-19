#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>
#include <osqp/osqp.h>  // v0.6.x のヘッダ
#include <Eigen/Sparse>

#include <algorithm>
#include <limits>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  if (!subscribeMessageAvailable()) return;

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  if ((closet_traj_point_idx == trajectory_->points.size() - 1) ||
      (trajectory_->points.size() <= 2)) {
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -10.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "reached to the goal");
  } else {
    const auto & pt = trajectory_->points.at(closet_traj_point_idx);
    double current_vel = odometry_->twist.twist.linear.x;
    double target_vel = use_external_target_vel_ ? external_target_vel_ : pt.longitudinal_velocity_mps;

    // QP定義: minimize 0.5 * x' P x + q' x
    Eigen::SparseMatrix<float> P(2, 2);
    P.insert(0, 0) = 2.0;
    P.insert(1, 1) = 2.0;

    Eigen::VectorXd q(2);
    q << -2.0 * target_vel, 0.0;

    Eigen::SparseMatrix<float> A(4, 2);
    A.insert(0, 0) = 1.0;  // v
    A.insert(1, 1) = 1.0;  // delta
    A.insert(2, 0) = 1.0;  // v
    A.insert(3, 1) = 1.0;  // delta

    Eigen::VectorXd l(4), u(4);
    l << -5.0, -10.0, -OSQP_INFTY, -OSQP_INFTY;
    u <<  OSQP_INFTY, OSQP_INFTY, 20.0, 10.0;

    // 設定
    OSQPSettings *settings = OSQPSettings_new();
    osqp_settings_set_default(settings);
    settings->verbose = false;

    // モデル生成
    OSQPModel *boost::geometry::model = osqp_codegen();
    osqp_model_set_data(boost::geometry::model, P, q, A, l, u);
    OSQPSolver *solver = osqp_solver_new_from_model(boost::geometry::model, settings);

    osqp_solver_solve(solver);
    const OSQPSolution *sol = osqp_get_solution(solver);

    double v_out = current_vel;
    double delta_out = 0.0;

    if (sol && sol->status_val == OSQP_SOLVED) {
      v_out = sol->x[0];
      delta_out = sol->x[1];
    }

    osqp_cleanup(solver);
    osqp_model_free(boost::geometry::model);
    OSQPSettings_free(settings);

    cmd.longitudinal.speed = v_out;
    cmd.longitudinal.acceleration = speed_proportional_gain_ * (v_out - current_vel);
    cmd.lateral.steering_tire_angle = delta_out;
  }

  pub_cmd_->publish(cmd);
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "odometry is not available");
    return false;
  }
  if (!trajectory_ || trajectory_->points.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "trajectory is not available");
    return false;
  }
  return true;
}

}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
