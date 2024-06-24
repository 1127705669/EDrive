/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/controller/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/LU"

#include "common/configs/vehicle_config_helper.h"
#include "common/src/log.h"
#include "common/math/math_utils.h"
#include "common/math/mpc_solver.h"

#include "common/src/log.h"

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
  ROS_INFO("    registering MPC controller...");
}

void MPCController::LoadControlCalibrationTable(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &control_table = mpc_controller_conf.calibration_table();
  
  ROS_INFO("Control calibration table loaded");
  ROS_INFO("Control calibration table size is %d", control_table.calibration_size());

  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  if(!control_interpolation_->Init(xyz)){
    ROS_ERROR("Fail to load control calibration table");
  }
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

void MPCController::LoadMPCGainScheduler(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &lat_err_gain_scheduler =
      mpc_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      mpc_controller_conf.heading_err_gain_scheduler();
  const auto &feedforwardterm_gain_scheduler =
      mpc_controller_conf.feedforwardterm_gain_scheduler();
  const auto &steer_weight_gain_scheduler =
      mpc_controller_conf.steer_weight_gain_scheduler();
  EINFO("MPC control gain scheduler loaded");
  Interpolation1D::DataType xy1, xy2, xy3, xy4;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : feedforwardterm_gain_scheduler.scheduler()) {
    xy3.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : steer_weight_gain_scheduler.scheduler()) {
    xy4.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  ECHECK(lat_err_interpolation_->Init(xy1));

  heading_err_interpolation_.reset(new Interpolation1D);
  ECHECK(heading_err_interpolation_->Init(xy2));

  feedforwardterm_interpolation_.reset(new Interpolation1D);
  ECHECK(feedforwardterm_interpolation_->Init(xy2));

  steer_weight_interpolation_.reset(new Interpolation1D);
  ECHECK(steer_weight_interpolation_->Init(xy2));
}

Result_state MPCController::ComputeControlCommand(
    const ::planning::ADCTrajectory *trajectory,
    const nav_msgs::Odometry *localization,
    ControlCommand *control_command) {
  double vx = localization->twist.twist.linear.x;
  double vy = localization->twist.twist.linear.y;
  double velocity_magnitude = std::sqrt(vx * vx + vy * vy);
  VehicleStateProvider::instance()->set_linear_velocity(velocity_magnitude);

  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(trajectory));

  SimpleMPCDebug *debug = control_command->mutable_debug()->mutable_simple_mpc_debug();
  debug->Clear();

  ComputeLongitudinalErrors(&trajectory_analyzer_, debug);

  // Update state
  UpdateStateAnalyticalMatching(debug);

  UpdateMatrix(debug);

  FeedforwardUpdate(debug);

  // Add gain sheduler for higher speed steering
  matrix_q_updated_ = matrix_q_;
  matrix_r_updated_ = matrix_r_;
  steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_;

  debug->add_matrix_q_updated(matrix_q_updated_(0, 0));
  debug->add_matrix_q_updated(matrix_q_updated_(1, 1));
  debug->add_matrix_q_updated(matrix_q_updated_(2, 2));
  debug->add_matrix_q_updated(matrix_q_updated_(3, 3));

  debug->add_matrix_r_updated(matrix_r_updated_(0, 0));
  debug->add_matrix_r_updated(matrix_r_updated_(1, 1));

  Eigen::MatrixXd control_matrix(controls_, 1);
  control_matrix << 0, 0;

  Eigen::MatrixXd reference_state(basic_state_size_, 1);
  reference_state << 0, 0, 0, 0, 0, 0;

  std::vector<Eigen::MatrixXd> reference(horizon_, reference_state);

  Eigen::MatrixXd lower_bound(controls_, 1);
  lower_bound << -steer_single_direction_max_degree_, max_deceleration_;

  Eigen::MatrixXd upper_bound(controls_, 1);
  upper_bound << steer_single_direction_max_degree_, max_acceleration_;

  std::vector<Eigen::MatrixXd> control(horizon_, control_matrix);

  ros::Time mpc_start_timestamp = ros::Time::now();

  if (common::math::SolveLinearMPC(
          matrix_ad_, matrix_bd_, matrix_cd_, matrix_q_updated_,
          matrix_r_updated_, lower_bound, upper_bound, matrix_state_, reference,
          mpc_eps_, mpc_max_iteration_, &control) != true) {
    EERROR("MPC solver failed");
  } else {
    // EINFO("MPC problem solved! ");
  }

  ros::Time mpc_end_timestamp = ros::Time::now();

  // TODO(QiL): evaluate whether need to add spline smoothing after the result

  double steer_angle_feedback = control[0](0, 0) * 180 / M_PI *
                                steer_transmission_ratio_ /
                                steer_single_direction_max_degree_ * 100;
  double steer_angle =
      steer_angle_feedback + steer_angle_feedforwardterm_updated_;

  // Clamp the steer angle to -100.0 to 100.0
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

  steer_angle = digital_filter_.Filter(steer_angle);
    control_command->set_steering_target(steer_angle);

  debug->set_acceleration_cmd_closeloop(control[0](1, 0));

  double acceleration_cmd = control[0](1, 0) + debug->acceleration_reference();
  // TODO(QiL): add pitch angle feedforward to accomendate for 3D control

  debug->set_acceleration_cmd(acceleration_cmd);

  double calibration_value = 0.0;
  calibration_value = control_interpolation_->Interpolate(std::make_pair(
        VehicleStateProvider::instance()->linear_velocity(), acceleration_cmd));

  debug->set_calibration_value(calibration_value);

  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;
  if (calibration_value >= 0) {
    throttle_cmd = calibration_value > throttle_deadzone_ ? calibration_value
                                                          : throttle_deadzone_;
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    brake_cmd = -calibration_value > brake_deadzone_ ? -calibration_value
                                                     : brake_deadzone_;
  }

  control_command->set_steering_rate(100.0);
  control_command->set_throttle(throttle_cmd);
  control_command->set_brake(brake_cmd);

  debug->set_heading(VehicleStateProvider::instance()->heading());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforwardterm_updated_);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  // debug->set_steering_position(chassis->steering_percentage());

  // if (std::fabs(VehicleStateProvider::instance()->linear_velocity()) <=
  //         vehicle_param_.max_abs_speed_when_stopped() ||
  //     chassis->gear_location() == planning_published_trajectory->gear() ||
  //     chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
  //   control_command->set_gear_location(planning_published_trajectory->gear());
  // } else {
  //   control_command->set_gear_location(chassis->gear_location());
  // }

  return Result_state::State_Ok;
}

MPCController::~MPCController() {}

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    EERROR("[MPCController] control_conf == nullptr");
    return false;
  }
  const auto &vehicle_param_ =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();

  ts_ = control_conf->mpc_controller_conf().ts();
  ECHECK_GT(ts_, 0.0);
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

  LoadControlCalibrationTable(control_conf->mpc_controller_conf());

  ROS_INFO("MPC conf loaded");
  return true;
}

void MPCController::LogInitParameters() {
  // AINFO << name_ << " begin.";
  // AINFO << "[MPCController parameters]"
  //       << " mass_: " << mass_ << ","
  //       << " iz_: " << iz_ << ","
  //       << " lf_: " << lf_ << ","
  //       << " lr_: " << lr_;
}

void MPCController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->mpc_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(
      control_conf->mpc_controller_conf().mean_filter_window_size());
  heading_error_filter_ = common::MeanFilter(
      control_conf->mpc_controller_conf().mean_filter_window_size());
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

  int q_param_size = control_conf->mpc_controller_conf().matrix_q_size();
  // if (basic_state_size_ != q_param_size) {
  //   const auto error_msg = common::util::StrCat(
  //       "MPC controller error: matrix_q size: ", q_param_size,
  //       " in parameter file not equal to basic_state_size_: ",
  //       basic_state_size_);
  //   AERROR << error_msg;
  //   return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  // }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  }

  // Update matrix_q_updated_ and matrix_r_updated_
  matrix_r_updated_ = matrix_r_;
  matrix_q_updated_ = matrix_q_;

  InitializeFilters(control_conf);
  LoadMPCGainScheduler(control_conf->mpc_controller_conf());
  LogInitParameters();
  ROS_INFO("[MPCController] init done!");
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

void MPCController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, SimpleMPCDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  const auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      VehicleStateProvider::instance()->x(),
      VehicleStateProvider::instance()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      VehicleStateProvider::instance()->x(),
      VehicleStateProvider::instance()->y(),
      VehicleStateProvider::instance()->heading(),
      VehicleStateProvider::instance()->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

  ros::Time current_control_time = ros::Time::now();

  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time.toSec());

  // ADEBUG << "matched point:" << matched_point.DebugString();
  // ADEBUG << "reference point:" << reference_point.DebugString();
  debug->set_station_error(reference_point.path_point.s - s_matched);
  debug->set_speed_error(reference_point.v - s_dot_matched);

  debug->set_station_reference(reference_point.path_point.s);
  debug->set_speed_reference(reference_point.v);
  debug->set_acceleration_reference(reference_point.a);

  debug->set_station_feedback(s_matched);
  debug->set_speed_feedback(
      VehicleStateProvider::instance()->linear_velocity());
}

}  // namespace control
}  // namespace EDrive