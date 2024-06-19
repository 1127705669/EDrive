/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include <algorithm> 

#include "control/src/controller/lat_controller.h"

#include "common/src/configs/vehicle_config_helper.h"

namespace EDrive {
namespace control {

using EDrive::common::Result_state;
using Matrix = Eigen::MatrixXd;
using EDrive::common::VehicleStateProvider;
using ::common::TrajectoryPoint;

constexpr double GRA_ACC = 9.8;

LatController::LatController() : name_("LQR-based Lateral Controller") {
  ROS_INFO("    registering Lat controller...");
}

LatController::~LatController() {
  
}

Result_state LatController::Init(const ControlConf *control_conf) {
  if (!LoadControlConf(control_conf)) {
    ROS_ERROR("failed to load control conf");
  }
  // Matrix init operations.
  const int matrix_size = basic_state_size_ + preview_window_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  matrix_r_ = Matrix::Identity(1, 1);
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

  int q_param_size = control_conf->lat_controller_conf().matrix_q_size();

  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->lat_controller_conf().matrix_q(i);
  }

  matrix_q_updated_ = matrix_q_;
  
  auto &lat_controller_conf = control_conf->lat_controller_conf();

  return Result_state::State_Ok;
}

bool LatController::LoadControlConf(const ControlConf *control_conf) {

  if (!control_conf) {
    ROS_ERROR("[LatController] control_conf == nullptr");
    return false;
  }

  const auto &vehicle_param_ =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();

  ts_ = control_conf->lat_controller_conf().ts();
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_transmission_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();

  const double mass_fl = control_conf->lat_controller_conf().mass_fl();
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();

  query_relative_time_ = control_conf->query_relative_time();

  minimum_speed_protection_ = control_conf->minimum_speed_protection();

  return true;
}

std::string LatController::Name() const { return name_; }

Result_state LatController::ComputeControlCommand(
    const ::planning::ADCTrajectory *trajectory,
    const nav_msgs::Odometry *localization,
    ::control::CarlaEgoVehicleControl *control_command) {
  if (trajectory_analyzer_ == nullptr) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory));
  }

  SimpleLateralDebug *debug;

  UpdateStateAnalyticalMatching(debug);

  UpdateMatrix();

  UpdateMatrixCompound();

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio
  const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                      M_PI * steer_transmission_ratio_ /
                                      steer_single_direction_max_degree_ * 100;

  return Result_state::State_Ok;
}

void LatController::UpdateStateAnalyticalMatching(SimpleLateralDebug *debug) {

  const auto &com = VehicleStateProvider::instance()->ComputeCOMPosition(lr_);

  ComputeLateralErrors(com.x(), com.y(),
                       VehicleStateProvider::instance()->linear_velocity(),
                       VehicleStateProvider::instance()->heading(),
                       VehicleStateProvider::instance()->angular_velocity(),
                       *trajectory_analyzer_, debug);

  // State matrix update;
  // First four elements are fixed;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
}

void LatController::UpdateMatrix(){
  const double v = std::max(VehicleStateProvider::instance()->linear_velocity(),
                            minimum_speed_protection_);
}

void LatController::UpdateMatrixCompound(){

}

Result_state LatController::Reset(){
  return Result_state::State_Ok;
}

void LatController::Stop() {
  ROS_INFO("stop");
}

void LatController::ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug) {
  // TODO(QiL): change this to conf.
  TrajectoryPoint target_point;

  target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);

  const double dx = x - target_point.path_point.x;
  const double dy = y - target_point.path_point.y;

  const double cos_matched_theta = std::cos(target_point.path_point.theta);
  const double sin_matched_theta = std::sin(target_point.path_point.theta);

  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  // lateral_error_ = lateral_rate_filter_.Filter(raw_lateral_error);

  // TODO(QiL): Code reformat when done with test
  const double raw_lateral_error =
      cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(raw_lateral_error);
}

} // namespace control
} // namespace EDrive