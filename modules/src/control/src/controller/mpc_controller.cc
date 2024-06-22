/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/controller/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/LU"

#include "common/configs/vehicle_config_helper.h"
#include "common/math/math_utils.h"

namespace EDrive {
namespace control {

using EDrive::common::Result_state;
using EDrive::common::VehicleStateProvider;
using Matrix = Eigen::MatrixXd;
using ::common::TrajectoryPoint;
using EDrive::common::VehicleConfigHelper;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "/tmp/mpc_controller_%F_%H%M%S.csv",
           localtime(&raw_time));
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {}
}  // namespace

MPCController::MPCController() : name_("MPC Controller") {
  ROS_INFO("    registering Lat controller...");
}

void MPCController::UpdateStateAnalyticalMatching(SimpleMPCDebug *debug) {
  const auto &com = VehicleStateProvider::instance()->ComputeCOMPosition(lr_);
  ComputeLateralErrors(com.x(), com.y(),
                       VehicleStateProvider::instance()->heading(),
                       VehicleStateProvider::instance()->linear_velocity(),
                       VehicleStateProvider::instance()->angular_velocity(),
                       trajectory_analyzer_, debug);

  // State matrix update;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
  matrix_state_(4, 0) = debug->station_error();
  matrix_state_(5, 0) = debug->speed_error();
}

void MPCController::UpdateMatrix(SimpleMPCDebug *debug) {
  const double v = std::max(VehicleStateProvider::instance()->linear_velocity(),
                            minimum_speed_protection_);
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);

  matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / v - v;
  matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / v;
  matrix_cd_ = matrix_c_ * debug->heading_error_rate() * ts_;
}

void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const TrajectoryAnalyzer &trajectory_analyzer,
    SimpleMPCDebug *debug) {
  const auto matched_point =
      trajectory_analyzer.QueryNearestPointByPosition(x, y);

  const double dx = x - matched_point.path_point.x;
  const double dy = y - matched_point.path_point.y;

  const double cos_matched_theta = std::cos(matched_point.path_point.theta);
  const double sin_matched_theta = std::sin(matched_point.path_point.theta);
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(cos_matched_theta * dy - sin_matched_theta * dx);

  const double delta_theta =
      EDrive::common::math::NormalizeAngle(theta - matched_point.path_point.theta);
  const double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = chassis_v * sin_delta_theta;
  debug->set_lateral_error_rate(linear_v * sin_delta_theta);

  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  // theta_error_dot = angular_v - matched_point.path_point().kappa() *
  // matched_point.v();
  debug->set_heading_error_rate(angular_v - matched_point.path_point.kappa *
                                                matched_point.v);

  // matched_theta = matched_point.path_point().theta();
  debug->set_ref_heading(matched_point.path_point.theta);
  // matched_kappa = matched_point.path_point().kappa();
  debug->set_curvature(matched_point.path_point.kappa);
}

std::string MPCController::Name() const { return name_; }

Result_state MPCController::ComputeControlCommand(
    const ::planning::ADCTrajectory *trajectory,
    const nav_msgs::Odometry *localization,
    ::control::CarlaEgoVehicleControl *control_command) {



  return Result_state::State_Ok;
}

MPCController::~MPCController() {}

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    ROS_ERROR("[MPCController] control_conf == nullptr");
    return false;
  }
  const auto &vehicle_param_ =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();

  ts_ = control_conf->mpc_controller_conf().ts();
  // CHECK_GT(ts_, 0.0) << "[MPCController] Invalid control update interval.";
  cf_ = control_conf->mpc_controller_conf().cf();
  cr_ = control_conf->mpc_controller_conf().cr();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_transmission_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / steer_transmission_ratio_ / 180 * M_PI;
  max_lat_acc_ = control_conf->mpc_controller_conf().max_lateral_acceleration();
  max_acceleration_ = vehicle_param_.max_acceleration();
  max_deceleration_ = vehicle_param_.max_deceleration();

  const double mass_fl = control_conf->mpc_controller_conf().mass_fl();
  const double mass_fr = control_conf->mpc_controller_conf().mass_fr();
  const double mass_rl = control_conf->mpc_controller_conf().mass_rl();
  const double mass_rr = control_conf->mpc_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  mpc_eps_ = control_conf->mpc_controller_conf().eps();
  mpc_max_iteration_ = control_conf->mpc_controller_conf().max_iteration();
  throttle_deadzone_ = control_conf->mpc_controller_conf().throttle_deadzone();
  brake_deadzone_ = control_conf->mpc_controller_conf().brake_deadzone();

  minimum_speed_protection_ = control_conf->minimum_speed_protection();
  standstill_acceleration_ =
      control_conf->mpc_controller_conf().standstill_acceleration();

  // LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  // AINFO << "MPC conf loaded";
  return true;
}

Result_state MPCController::Init(const ControlConf *control_conf) {
  if (!LoadControlConf(control_conf)) {
    ROS_ERROR("failed to load control conf");
    return Result_state::State_Failed;
  }
  // Matrix init operations.
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(4, 5) = 1.0;
  matrix_a_(5, 5) = 0.0;
  // TODO(QiL): expand the model to accomendate more combined states.

  matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_b_(4, 1) = 0.0;
  matrix_b_(5, 1) = -1.0;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_c_ = Matrix::Zero(basic_state_size_, 1);
  matrix_c_(5, 0) = 1.0;
  matrix_cd_ = Matrix::Zero(basic_state_size_, 1);

  matrix_state_ = Matrix::Zero(basic_state_size_, 1);
  matrix_k_ = Matrix::Zero(1, basic_state_size_);

  matrix_r_ = Matrix::Identity(controls_, controls_);

  matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

  int r_param_size = control_conf->mpc_controller_conf().matrix_r_size();
  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf->mpc_controller_conf().matrix_r(i);
  }

  // int q_param_size = control_conf->mpc_controller_conf().matrix_q_size();
  // if (basic_state_size_ != q_param_size) {
  //   const auto error_msg = common::util::StrCat(
  //       "MPC controller error: matrix_q size: ", q_param_size,
  //       " in parameter file not equal to basic_state_size_: ",
  //       basic_state_size_);
  //   AERROR << error_msg;
  //   return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  // }
  // for (int i = 0; i < q_param_size; ++i) {
  //   matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  // }

  // // Update matrix_q_updated_ and matrix_r_updated_
  // matrix_r_updated_ = matrix_r_;
  // matrix_q_updated_ = matrix_q_;

  // InitializeFilters(control_conf);
  // LoadMPCGainScheduler(control_conf->mpc_controller_conf());
  // LogInitParameters();
  // AINFO << "[MPCController] init done!";
  return Result_state::State_Ok;
}

Result_state MPCController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  return Result_state::State_Ok;
}

void MPCController::CloseLogFile() {
  
}

void MPCController::Stop() {
  CloseLogFile();
}

/*
 * SL coordinate system:
 *  left to the ref_line, L is +
 * right to the ref_line, L is -
 */
double MPCController::GetLateralError(const common::math::Vec2d &point,
                                      TrajectoryPoint *traj_point) const {
  const auto closest =
      trajectory_analyzer_.QueryNearestPointByPosition(point.x(), point.y());

  const double point_angle = std::atan2(point.y() - closest.path_point.y,
                                        point.x() - closest.path_point.x);
  const double point2path_angle = point_angle - closest.path_point.theta;
  if (traj_point != nullptr) {
    *traj_point = closest;
  }

  const double dx = closest.path_point.x - point.x();
  const double dy = closest.path_point.y - point.y();
  return std::sin(point2path_angle) * std::sqrt(dx * dx + dy * dy);
}

void MPCController::FeedforwardUpdate(SimpleMPCDebug *debug) {
  steer_angle_feedforwardterm_ = (wheelbase_ * debug->curvature()) * 180 /
                                 M_PI * steer_transmission_ratio_ /
                                 steer_single_direction_max_degree_ * 100;
}

}  // namespace control
}  // namespace EDrive