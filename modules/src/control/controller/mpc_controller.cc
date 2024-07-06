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
#include "common/math/mpc_osqp.h"
#include "control/common/control_gflags.h"

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
  EINFO << "    registering MPC controller...";
}

void MPCController::LoadControlCalibrationTable(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &control_table = mpc_controller_conf.calibration_table();
  EDEBUG << "Control calibration table loaded";
  EDEBUG << "Control calibration table size is " << control_table.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  if(!control_interpolation_->Init(xyz)){
    EERROR << "Fail to load control calibration table";
  }
}

void MPCController::UpdateState(SimpleMPCDebug *debug) {
  const auto &com = VehicleStateProvider::Instance()->ComputeCOMPosition(lr_);
  ComputeLateralErrors(com.x(), com.y(),
                       VehicleStateProvider::Instance()->heading(),
                       VehicleStateProvider::Instance()->linear_velocity(),
                       VehicleStateProvider::Instance()->angular_velocity(),
                       VehicleStateProvider::Instance()->linear_acceleration(),
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
  const double v = std::max(VehicleStateProvider::Instance()->linear_velocity(),
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
  matrix_cd_ = matrix_c_ * debug->ref_heading_rate() * ts_;
}

void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleMPCDebug *debug) {
  const auto matched_point =
      trajectory_analyzer.QueryNearestPointByPosition(VehicleStateProvider::Instance()->x(), VehicleStateProvider::Instance()->y());

  const double dx = VehicleStateProvider::Instance()->x() - matched_point.path_point.x;
  const double dy = VehicleStateProvider::Instance()->y() - matched_point.path_point.y;

  const double cos_matched_theta = std::cos(matched_point.path_point.theta);
  const double sin_matched_theta = std::sin(matched_point.path_point.theta);
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(cos_matched_theta * dy - sin_matched_theta * dx);

  // matched_theta = matched_point.path_point().theta();
  debug->set_ref_heading(matched_point.path_point.theta);
  const double delta_theta =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  debug->set_heading_error(delta_theta);

  const double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = chassis_v * sin_delta_theta;
  double lateral_error_dot = linear_v * sin_delta_theta;
  double lateral_error_dot_dot = linear_a * sin_delta_theta;

  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  // matched_kappa = matched_point.path_point().kappa();
  debug->set_curvature(matched_point.path_point.kappa);
  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  // theta_error_dot = angular_v - matched_point.path_point().kappa() *
  // matched_point.v();
  debug->set_heading_rate(angular_v);
  debug->set_ref_heading_rate(debug->curvature() * matched_point.v);
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();
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
  EDEBUG << "MPC control gain scheduler loaded";
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
    ControlCommand *cmd) {
  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(trajectory));

  if (control_conf_.trajectory_transform_to_com_reverse()) {
    trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
  }

  SimpleMPCDebug *debug = cmd->mutable_debug()->mutable_simple_mpc_debug();
  debug->Clear();

  ComputeLongitudinalErrors(&trajectory_analyzer_, debug);

  // Update state
  UpdateState(debug);

  UpdateMatrix(debug);

  FeedforwardUpdate(debug);

  matrix_q_updated_ = matrix_q_;
  matrix_r_updated_ = matrix_r_;
  steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_;

  debug->add_matrix_q_updated(matrix_q_updated_(0, 0));
  debug->add_matrix_q_updated(matrix_q_updated_(1, 1));
  debug->add_matrix_q_updated(matrix_q_updated_(2, 2));
  debug->add_matrix_q_updated(matrix_q_updated_(3, 3));

  debug->add_matrix_r_updated(matrix_r_updated_(0, 0));
  debug->add_matrix_r_updated(matrix_r_updated_(1, 1));

  Matrix control_matrix = Matrix::Zero(controls_, 1);
  std::vector<Matrix> control(horizon_, control_matrix);

  Matrix control_gain_matrix = Matrix::Zero(controls_, basic_state_size_);
  std::vector<Matrix> control_gain(horizon_, control_gain_matrix);

  Matrix addition_gain_matrix = Matrix::Zero(controls_, 1);
  std::vector<Matrix> addition_gain(horizon_, addition_gain_matrix);

  Matrix reference_state = Matrix::Zero(basic_state_size_, 1);
  std::vector<Matrix> reference(horizon_, reference_state);

  Matrix lower_bound(controls_, 1);
  lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;

  Matrix upper_bound(controls_, 1);
  upper_bound << wheel_single_direction_max_degree_, max_acceleration_;

  const double max = std::numeric_limits<double>::max();
  Matrix lower_state_bound(basic_state_size_, 1);
  Matrix upper_state_bound(basic_state_size_, 1);

  // lateral_error, lateral_error_rate, heading_error, heading_error_rate
  // station_error, station_error_rate
  lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
      -1.0 * max, -1.0 * max;
  upper_state_bound << max, max, M_PI, max, max, max;

  ros::Time mpc_start_timestamp = ros::Time::now();
  double steer_angle_feedback = 0.0;
  double acc_feedback = 0.0;
  double steer_angle_ff_compensation = 0.0;
  double unconstrained_control_diff = 0.0;
  double control_gain_truncation_ratio = 0.0;
  double unconstrained_control = 0.0;
  const double v = VehicleStateProvider::Instance()->linear_velocity();

  std::vector<double> control_cmd(controls_, 0);

  EDrive::common::math::MpcOsqp mpc_osqp(
      matrix_ad_, matrix_bd_, matrix_q_updated_, matrix_r_updated_,
      matrix_state_, lower_bound, upper_bound, lower_state_bound,
      upper_state_bound, reference_state, mpc_max_iteration_, horizon_,
      mpc_eps_);
  if (!mpc_osqp.Solve(&control_cmd)) {
    EERROR << "MPC OSQP solver failed";
  } else {
    EDEBUG << "MPC OSQP problem solved! ";
    control[0](0, 0) = control_cmd.at(0);
    control[0](1, 0) = control_cmd.at(1);
  }

  steer_angle_feedback = Wheel2SteerPct(control[0](0, 0));
  acc_feedback = control[0](1, 0);
  for (int i = 0; i < basic_state_size_; ++i) {
    unconstrained_control += control_gain[0](0, i) * matrix_state_(i, 0);
  }
  unconstrained_control += addition_gain[0](0, 0) * v * debug->curvature();
  if (enable_mpc_feedforward_compensation_) {
    unconstrained_control_diff =
        Wheel2SteerPct(control[0](0, 0) - unconstrained_control);
    if (fabs(unconstrained_control_diff) <= unconstrained_control_diff_limit_) {
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v));
    } else {
      control_gain_truncation_ratio = control[0](0, 0) / unconstrained_control;
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v) *
                         control_gain_truncation_ratio);
    }
  } else {
    steer_angle_ff_compensation = 0.0;
  }

  ros::Time mpc_end_timestamp = ros::Time::now();

  EDEBUG << "MPC core algorithm: calculation time is: "
         << (mpc_end_timestamp.toSec() - mpc_start_timestamp.toSec()) * 1000 << " ms.";

  // TODO(QiL): evaluate whether need to add spline smoothing after the result
  double steer_angle = steer_angle_feedback +
                       steer_angle_feedforwardterm_updated_ +
                       steer_angle_ff_compensation;

  if (FLAGS_set_steer_limit) {
    const double steer_limit = std::atan(max_lat_acc_ * wheelbase_ /
                                         (VehicleStateProvider::Instance()->linear_velocity() *
                                          VehicleStateProvider::Instance()->linear_velocity())) *
                               steer_ratio_ * 180 / M_PI /
                               steer_single_direction_max_degree_ * 100;

    // Clamp the steer angle with steer limitations at current speed
    double steer_angle_limited =
        common::math::Clamp(steer_angle, -steer_limit, steer_limit);
    steer_angle_limited = digital_filter_.Filter(steer_angle_limited);
    steer_angle = steer_angle_limited;
    debug->set_steer_angle_limited(steer_angle_limited);
  }

  steer_angle = digital_filter_.Filter(steer_angle);
  // Clamp the steer angle to -100.0 to 100.0
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);
  cmd->set_steering_target(steer_angle);

  debug->set_acceleration_cmd_closeloop(acc_feedback);

  double acceleration_cmd = acc_feedback + debug->acceleration_reference();
  // TODO(QiL): add pitch angle feed forward to accommodate for 3D control

  debug->set_acceleration_cmd(acceleration_cmd);

  double calibration_value = 0.0;
  calibration_value = control_interpolation_->Interpolate(std::make_pair(
        VehicleStateProvider::Instance()->linear_velocity(), acceleration_cmd));

  debug->set_calibration_value(calibration_value);

  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;
  if (calibration_value >= 0) {
    throttle_cmd = std::max(calibration_value, throttle_lowerbound_);
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    brake_cmd = std::max(-calibration_value, brake_lowerbound_);
  }

  cmd->set_steering_rate(FLAGS_steer_angle_rate);
  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);

  debug->set_heading(VehicleStateProvider::Instance()->heading());
  // debug->set_steering_position(chassis->steering_percentage());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforwardterm_updated_);
  debug->set_steer_angle_feedforward_compensation(steer_angle_ff_compensation);
  debug->set_steer_unconstrained_control_diff(unconstrained_control_diff);
  debug->set_steer_angle_feedback(steer_angle_feedback);

  return Result_state::State_Ok;
}

MPCController::~MPCController() {}

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    EERROR << "[MPCController] control_conf == nullptr";
    return false;
  }
  vehicle_param_ = VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = control_conf->mpc_controller_conf().ts();
  if (ts_ <= 0.0) {
    EERROR << "[MPCController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->mpc_controller_conf().cf();
  cr_ = control_conf->mpc_controller_conf().cr();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() * 180 / M_PI;
  max_lat_acc_ = control_conf->mpc_controller_conf().max_lateral_acceleration();

  // steering ratio should be positive
  static constexpr double kEpsilon = 1e-6;
  if (std::isnan(steer_ratio_) || steer_ratio_ < kEpsilon) {
    EERROR << "[MPCController] steer_ratio = 0";
    return false;
  }
  wheel_single_direction_max_degree_ =
      steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;
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
  throttle_lowerbound_ =
      std::max(vehicle_param_.throttle_deadzone(),
               control_conf->mpc_controller_conf().throttle_minimum_action());
  brake_lowerbound_ =
      std::max(vehicle_param_.brake_deadzone(),
               control_conf->mpc_controller_conf().brake_minimum_action());

  minimum_speed_protection_ = FLAGS_minimum_speed_protection;
  max_acceleration_when_stopped_ = FLAGS_max_acceleration_when_stopped;
  max_abs_speed_when_stopped_ = vehicle_param_.max_abs_speed_when_stopped();
  standstill_acceleration_ =
      control_conf->mpc_controller_conf().standstill_acceleration();

  enable_mpc_feedforward_compensation_ =
      control_conf->mpc_controller_conf().enable_mpc_feedforward_compensation();

  unconstrained_control_diff_limit_ =
      control_conf->mpc_controller_conf().unconstrained_control_diff_limit();
    
  enable_look_ahead_back_control_ =
      control_conf->mpc_controller_conf().enable_look_ahead_back_control();

  LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  EINFO << "MPC conf loaded";
  return true;
}

void MPCController::LogInitParameters() {
  // EINFO << name_ << " begin.";
  // EINFO << "[MPCController parameters]"
  //       << " mass_: " << mass_ << ","
  //       << " iz_: " << iz_ << ","
  //       << " lf_: " << lf_ << ","
  //       << " lr_: " << lr_;

  EINFO << "Configuration Parameters:";
  EINFO << "Front cornering stiffness (cf_): " << cf_;
  EINFO << "Rear cornering stiffness (cr_): " << cr_;
  EINFO << "Wheelbase (wheelbase_): " << wheelbase_;
  EINFO << "Max steer angle (max_steer_angle): " << vehicle_param_.max_steer_angle();
  EINFO << "Steering transmission ratio (steer_ratio_): " << steer_ratio_;
  EINFO << "Max steering angle in degrees (steer_single_direction_max_degree_): " << steer_single_direction_max_degree_;
  EINFO << "Max wheel angle in degrees (wheel_single_direction_max_degree_): " << wheel_single_direction_max_degree_;
  EINFO << "Max lateral acceleration (max_lat_acc_): " << max_lat_acc_;
  EINFO << "Max acceleration (max_acceleration_): " << max_acceleration_;
  EINFO << "Max deceleration (max_deceleration_): " << max_deceleration_;
  EINFO << "Total mass (mass_): " << mass_;
  EINFO << "Front axle to center distance (lf_): " << lf_;
  EINFO << "Rear axle to center distance (lr_): " << lr_;
  EINFO << "Moment of inertia about Z-axis (iz_): " << iz_;
  EINFO << "MPC epsilon (mpc_eps_): " << mpc_eps_;
  EINFO << "MPC max iteration (mpc_max_iteration_): " << mpc_max_iteration_;
  EINFO << "Throttle deadzone (throttle_lowerbound_): " << throttle_lowerbound_;
  EINFO << "Brake deadzone (brake_lowerbound_): " << brake_lowerbound_;
  EINFO << "Minimum speed protection (minimum_speed_protection_): " << minimum_speed_protection_;
  EINFO << "Standstill acceleration (standstill_acceleration_): " << standstill_acceleration_;

}

void MPCController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->mpc_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
}

Result_state MPCController::Init(const ControlConf *control_conf) {
  if (!LoadControlConf(control_conf)) {
    ROS_ERROR("failed to load control conf");
    return Result_state::State_Failed;
  }
  control_conf_ = control_conf->mpc_controller_conf();
  // Matrix init operations.
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(4, 5) = 1.0;
  matrix_a_(5, 5) = 0.0;
  // TODO(QiL): expand the model to accommodate more combined states.

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
  if (basic_state_size_ != q_param_size) {
    // const auto error_msg = common::util::StrCat(
    //     "MPC controller error: matrix_q size: ", q_param_size,
    //     " in parameter file not equal to basic_state_size_: ",
    //     basic_state_size_);
    EERROR << "MPC controller error: matrix_q size: " << q_param_size;
    return Result_state::State_Failed;
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  }

  // Update matrix_q_updated_ and matrix_r_updated_
  matrix_r_updated_ = matrix_r_;
  matrix_q_updated_ = matrix_q_;

  InitializeFilters(control_conf);
  LoadMPCGainScheduler(control_conf->mpc_controller_conf());
  LogInitParameters();
  EDEBUG << "[MPCController] init done!";
  return Result_state::State_Ok;
}

Result_state MPCController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  return Result_state::State_Ok;
}

void MPCController::CloseLogFile() {
  
}

double MPCController::Wheel2SteerPct(const double wheel_angle) {
  return wheel_angle / wheel_single_direction_max_degree_ * 100;
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
  const double v = VehicleStateProvider::Instance()->linear_velocity();
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  steer_angle_feedforwardterm_ = Wheel2SteerPct(
      wheelbase_ * debug->curvature() + kv * v * v * debug->curvature());
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
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y(),
      VehicleStateProvider::Instance()->heading(),
      VehicleStateProvider::Instance()->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

  ros::Time current_control_time = ros::Time::now();

  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time.toSec());

  // EDEBUG << "matched point:" << matched_point.DebugString();
  // EDEBUG << "reference point:" << reference_point.DebugString();
  debug->set_station_error(reference_point.path_point.s - s_matched);
  debug->set_speed_error(reference_point.v - s_dot_matched);

  debug->set_station_reference(reference_point.path_point.s);
  debug->set_speed_reference(reference_point.v);
  debug->set_acceleration_reference(reference_point.a);

  debug->set_station_feedback(s_matched);
  debug->set_speed_feedback(
      VehicleStateProvider::Instance()->linear_velocity());
}

}  // namespace control
}  // namespace EDrive