/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include "control/controller/lat_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "common/src/log.h"

#include "common/configs/vehicle_config_helper.h"
// #include "common/math/linear_interpolation.h"
// #include "common/math/linear_quadratic_regulator.h"
// #include "common/math/math_utils.h"
// #include "common/math/quaternion.h"
#include "control/common/control_gflags.h"

namespace EDrive {
namespace control {

using EDrive::common::Result_state;
using Matrix = Eigen::MatrixXd;
using EDrive::common::VehicleStateProvider;
using ::common::TrajectoryPoint;

LatController::LatController() : name_("LQR-based Lateral Controller") {
  EINFO << "Using " << name_;
}

LatController::~LatController() { CloseLogFile(); }

void LatController::ProcessLogs(const SimpleLateralDebug *debug) {
  // const std::string log_str = absl::StrCat(
  //     debug->lateral_error(), ",", debug->ref_heading(), ",", debug->heading(),
  //     ",", debug->heading_error(), ",", debug->heading_error_rate(), ",",
  //     debug->lateral_error_rate(), ",", debug->curvature(), ",",
  //     debug->steer_angle(), ",", debug->steer_angle_feedforward(), ",",
  //     debug->steer_angle_lateral_contribution(), ",",
  //     debug->steer_angle_lateral_rate_contribution(), ",",
  //     debug->steer_angle_heading_contribution(), ",",
  //     debug->steer_angle_heading_rate_contribution(), ",",
  //     debug->steer_angle_feedback(), ",", chassis->steering_percentage(), ",",
  //     injector_->vehicle_state()->linear_velocity());
  // if (FLAGS_enable_csv_debug) {
  //   steer_log_file_ << log_str << std::endl;
  // }
  // ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatController::LogInitParameters() {
  EINFO << name_ << " begin.";
  EINFO << "[LatController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

void LatController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->lat_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
}

Result_state LatController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (!LoadControlConf(control_conf_)) {
    EERROR << "failed to load control conf";
  }
  // Matrix init operations.
  const int matrix_size = basic_state_size_ + preview_window_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
  /*
  A matrix (Gear Drive)
  [0.0, 1.0, 0.0, 0.0;
   0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m, (l_r * c_r - l_f * c_f) / m / v;
   0.0, 0.0, 0.0, 1.0;
   0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
  */
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
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

  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (matrix_size != q_param_size || matrix_size != reverse_q_param_size) {
    EERROR << "lateral controller error: matrix_q size: " << q_param_size;
    return Result_state::State_Failed;
  }

  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
  }

  matrix_q_updated_ = matrix_q_;
  InitializeFilters(control_conf_);
  auto &lat_controller_conf = control_conf_->lat_controller_conf();
  LoadLatGainScheduler(lat_controller_conf);
  LogInitParameters();

  return Result_state::State_Ok;
}

bool LatController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    EERROR << "[LatController] control_conf == nullptr";
    return false;
  }
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = control_conf->lat_controller_conf().ts();
  if (ts_ <= 0.0) {
    EERROR << "[LatController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  lookahead_station_low_speed_ =
      control_conf->lat_controller_conf().lookahead_station();
  lookback_station_low_speed_ =
      control_conf->lat_controller_conf().lookback_station();
  lookahead_station_high_speed_ =
      control_conf->lat_controller_conf().lookahead_station_high_speed();
  lookback_station_high_speed_ =
      control_conf->lat_controller_conf().lookback_station_high_speed();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();
  low_speed_bound_ = control_conf_->lon_controller_conf().switch_speed();
  low_speed_window_ =
      control_conf_->lon_controller_conf().switch_speed_window();

  const double mass_fl = control_conf->lat_controller_conf().mass_fl();
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  // moment of inertia
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();

  query_relative_time_ = control_conf->query_relative_time();

  minimum_speed_protection_ = control_conf->minimum_speed_protection();

  return true;
}

void LatController::CloseLogFile() {

}

void LatController::LoadLatGainScheduler(
    const LatControllerConf &lat_controller_conf) {
  const auto &lat_err_gain_scheduler =
      lat_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      lat_controller_conf.heading_err_gain_scheduler();
  EINFO << "Lateral control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  ECHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler";

  heading_err_interpolation_.reset(new Interpolation1D);
  ECHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler";
}

std::string LatController::Name() const { return name_; }

Result_state LatController::ComputeControlCommand(
    const ::planning::ADCTrajectory *trajectory,
    const nav_msgs::Odometry *localization,
    ControlCommand *cmd) {
  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(trajectory));

  /*
  A matrix (Gear Drive)
  [0.0, 1.0, 0.0, 0.0;
    0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m, (l_r * c_r - l_f * c_f) / m / v;
    0.0, 0.0, 0.0, 1.0;
    0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
  */
  cf_ = control_conf_->lat_controller_conf().cf();
  cr_ = control_conf_->lat_controller_conf().cr();
  matrix_a_(0, 1) = 1.0;
  matrix_a_coeff_(0, 2) = 0.0;

  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  UpdateDrivingOrientation();

  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  UpdateState(debug);

  UpdateMatrix();

  UpdateMatrixCompound();

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio
  const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                      M_PI * steer_ratio_ /
                                      steer_single_direction_max_degree_ * 100;

  return Result_state::State_Ok;
}

void LatController::UpdateState(SimpleLateralDebug *debug) {

  const auto &com = VehicleStateProvider::Instance()->ComputeCOMPosition(lr_);

  ComputeLateralErrors(
      com.x(), com.y(), driving_orientation_,
      VehicleStateProvider::Instance()->linear_velocity(), VehicleStateProvider::Instance()->angular_velocity(),
      VehicleStateProvider::Instance()->linear_acceleration(), trajectory_analyzer_, debug);

  // State matrix update;
  // First four elements are fixed;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  for (int i = 0; i < preview_window_; ++i) {
    const double preview_time = ts_ * (i + 1);
    const auto preview_point =
        trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);

    const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
        preview_point.path_point.x, preview_point.path_point.y);

    const double dx =
        preview_point.path_point.x - matched_point.path_point.x;
    const double dy =
        preview_point.path_point.y - matched_point.path_point.y;

    const double cos_matched_theta =
        std::cos(matched_point.path_point.theta);
    const double sin_matched_theta =
        std::sin(matched_point.path_point.theta);
    const double preview_d_error =
        cos_matched_theta * dy - sin_matched_theta * dx;

    matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
  }
}

void LatController::UpdateMatrix(){
  const double v = std::max(VehicleStateProvider::Instance()->linear_velocity(),
                            minimum_speed_protection_);
}

void LatController::UpdateMatrixCompound(){

}

Result_state LatController::Reset(){
  return Result_state::State_Ok;
}

void LatController::Stop() {
  EINFO << "stop";
}

double LatController::ComputeFeedForward(double ref_curvature) const {
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  // Calculate the feedforward term of the lateral controller; then change it
  // from rad to %
  const double v = VehicleStateProvider::Instance()->linear_velocity();
  double steer_angle_feedforwardterm;
  // if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE &&
  //     !lat_based_lqr_controller_conf_.reverse_use_dynamic_model()) {
  //   steer_angle_feedforwardterm =
  //       lat_based_lqr_controller_conf_.reverse_feedforward_ratio() *
  //       wheelbase_ * ref_curvature * 180 / M_PI * steer_ratio_ /
  //       steer_single_direction_max_degree_ * 100;
  // } else {
  //   steer_angle_feedforwardterm =
  //       (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
  //        matrix_k_(0, 2) *
  //            (lr_ * ref_curvature -
  //             lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
  //       180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
  // }

  return steer_angle_feedforwardterm;
}

void LatController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug) {
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

void LatController::UpdateDrivingOrientation() {
  // auto vehicle_state = injector_->vehicle_state();
  driving_orientation_ = VehicleStateProvider::Instance()->heading();
  matrix_bd_ = matrix_b_ * ts_;
  // Reverse the driving direction if the vehicle is in reverse mode
  // if (FLAGS_reverse_heading_control) {
  //   if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
  //     driving_orientation_ =
  //         common::math::NormalizeAngle(driving_orientation_ + M_PI);
  //     // Update Matrix_b for reverse mode
  //     matrix_bd_ = -matrix_b_ * ts_;
  //     ADEBUG << "Matrix_b changed due to gear direction";
  //   }
  // }
}

} // namespace control
} // namespace EDrive