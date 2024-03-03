/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "control/src/controller/lat_controller.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;
using Matrix = Eigen::MatrixXd;
using EDrive::common::VehicleStateProvider;

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

  matrix_q_updated_ = matrix_q_;
  
  auto &lat_controller_conf = control_conf->lat_controller_conf();

  return State_Ok;
}

bool LatController::LoadControlConf(const ControlConf *control_conf) {

  if (!control_conf) {
    ROS_ERROR("[LatController] control_conf == nullptr");
    return false;
  }

  ts_ = control_conf->lat_controller_conf().ts();
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  const double mass_fl = control_conf->lat_controller_conf().mass_fl();
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();

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

  ComputeLateralErrors(trajectory_analyzer_.get());

  SimpleLateralDebug *debug;

  UpdateStateAnalyticalMatching(debug);

  UpdateMatrix();

  UpdateMatrixCompound();

  return State_Ok;
}

void LatController::UpdateStateAnalyticalMatching(SimpleLateralDebug *debug) {
  // State matrix update;
  // First four elements are fixed;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
}

void LatController::UpdateMatrix(){

}

void LatController::UpdateMatrixCompound(){

}

Result_state LatController::Reset(){
  return State_Ok;
}

void LatController::Stop() {
  ROS_INFO("stop");
}

void LatController::ComputeLateralErrors(const TrajectoryAnalyzer *trajectory_analyzer) {
  
}

} // namespace control
} // namespace EDrive