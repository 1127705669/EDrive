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
#include "common/math/linear_interpolation.h"
#include "common/math/linear_quadratic_regulator.h"
#include "common/math/math_utils.h"
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
  // EDEBUG << "Steer_Control_Detail: " << log_str;
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
   0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
   (l_r * c_r - l_f * c_f) / m / v;
   0.0, 0.0, 0.0, 1.0;
   0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
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
    EERROR <<
        "lateral controller error: matrix_q size: ", q_param_size,
        "lateral controller error: reverse_matrix_q size: ",
        reverse_q_param_size,
        " in parameter file not equal to matrix_size: ", matrix_size;

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
  CHECK_GT(ts_, 0.0) << "[LatController] Invalid control update interval.";
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
  auto target_tracking_trajectory = *trajectory;

  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

  trajectory_analyzer_.TrajectoryTransformToCOM(lr_);

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

  // Adjust matrix_q_updated when in reverse gear
  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
  }

  // Add gain scheduler for higher speed steering
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) =
        matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(
                              std::fabs(VehicleStateProvider::Instance()->linear_velocity()));
    matrix_q_updated_(2, 2) =
        matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                              std::fabs(VehicleStateProvider::Instance()->linear_velocity()));
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  } else {
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  }

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio
  const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                      M_PI * steer_ratio_ /
                                      steer_single_direction_max_degree_ * 100;

  const double steer_angle_feedforward = ComputeFeedForward(debug->curvature());

  

  double steer_angle = 0.0;
  double steer_angle_feedback_augment = 0.0;
  // Augment the feedback control on lateral error at the desired speed domain
  if (enable_leadlag_) {
    if (FLAGS_enable_feedback_augment_on_high_speed ||
        std::fabs(VehicleStateProvider::Instance()->linear_velocity()) < low_speed_bound_) {
      steer_angle_feedback_augment =
          leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI *
          steer_ratio_ / steer_single_direction_max_degree_ * 100;
      if (std::fabs(VehicleStateProvider::Instance()->linear_velocity()) >
          low_speed_bound_ - low_speed_window_) {
        // Within the low-high speed transition window, linerly interplolate the
        // augment control gain for "soft" control switch
        steer_angle_feedback_augment = common::math::lerp(
            steer_angle_feedback_augment, low_speed_bound_ - low_speed_window_,
            0.0, low_speed_bound_, std::fabs(VehicleStateProvider::Instance()->linear_velocity()));
      }
    }
  }

  steer_angle = steer_angle_feedback + steer_angle_feedforward +
                steer_angle_feedback_augment;

  EINFO << "steer_angle_feedforward: " << steer_angle_feedforward << "   steer_angle: " << steer_angle;

  // Compute the steering command limit with the given maximum lateral
  // acceleration
  const double steer_limit =
      FLAGS_set_steer_limit ? std::atan(max_lat_acc_ * wheelbase_ /
                                        (VehicleStateProvider::Instance()->linear_velocity() *
                                         VehicleStateProvider::Instance()->linear_velocity())) *
                                  steer_ratio_ * 180 / M_PI /
                                  steer_single_direction_max_degree_ * 100
                            : 100.0;

  const double steer_diff_with_max_rate =
      FLAGS_enable_maximum_steer_rate_limit
          ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI /
                steer_single_direction_max_degree_ * 100
          : 100.0;

  // pre_steering_position_ = steering_position;
  // debug->set_steer_mrac_enable_status(enable_mrac_);

  // Clamp the steer angle with steer limitations at current speed
  double steer_angle_limited =
      common::math::Clamp(steer_angle, -steer_limit, steer_limit);
  steer_angle = steer_angle_limited;
  debug->set_steer_angle_limited(steer_angle_limited);

  // Limit the steering command with the designed digital filter
  steer_angle = digital_filter_.Filter(steer_angle);
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

  // Set the steer commands
  cmd->set_steering_target(common::math::Clamp(
      steer_angle, pre_steer_angle_ - steer_diff_with_max_rate,
      pre_steer_angle_ + steer_diff_with_max_rate));
  cmd->set_steering_rate(FLAGS_steer_angle_rate);

  pre_steer_angle_ = cmd->steering_target();

  // compute extra information for logging and debugging
  const double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_ratio_ /
      steer_single_direction_max_degree_ * 100;

  debug->set_heading(driving_orientation_);
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforward);
  debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
  debug->set_steer_angle_lateral_rate_contribution(
      steer_angle_lateral_rate_contribution);
  debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
  debug->set_steer_angle_heading_rate_contribution(
      steer_angle_heading_rate_contribution);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  // debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
  // debug->set_steering_position(steering_position);
  debug->set_ref_speed(VehicleStateProvider::Instance()->linear_velocity());
  
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

void LatController::UpdateMatrix() {
  double v;
  

  v = std::max(VehicleStateProvider::Instance()->linear_velocity(),
                minimum_speed_protection_);
  matrix_a_(0, 2) = 0.0;

  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LatController::UpdateMatrixCompound(){
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
  if (preview_window_ > 0) {
    matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
    // Update A matrix;
    for (int i = 0; i < preview_window_ - 1; ++i) {
      matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
    }
  }
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
  
  steer_angle_feedforwardterm =
      (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
        matrix_k_(0, 2) *
            (lr_ * ref_curvature -
            lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
      180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;

  return -steer_angle_feedforwardterm;
}

void LatController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug) {
  TrajectoryPoint target_point;

  target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);

  // EINFO << "location: " << x << ", " << y;
  // EINFO << "match point location: " << target_point.path_point.x << ", " << target_point.path_point.y;

  const double dx = x - target_point.path_point.x;
  const double dy = y - target_point.path_point.y;

  debug->mutable_current_target_point()->mutable_path_point()->set_x(
      target_point.path_point.x);
  debug->mutable_current_target_point()->mutable_path_point()->set_y(
      target_point.path_point.y);

  EDEBUG << "x point: " << x << " y point: " << y;
  // EDEBUG << "match point information : " << target_point.ShortDebugString();

  const double cos_target_heading = std::cos(target_point.path_point.theta);
  const double sin_target_heading = std::sin(target_point.path_point.theta);

  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
  if (FLAGS_enable_navigation_mode_error_filter) {
    lateral_error = lateral_error_filter_.Update(lateral_error);
  }

  debug->set_lateral_error(lateral_error);

  debug->set_ref_heading(target_point.path_point.theta);
  double heading_error =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  if (FLAGS_enable_navigation_mode_error_filter) {
    heading_error = heading_error_filter_.Update(heading_error);
  }
  debug->set_heading_error(heading_error);

  // Within the low-high speed transition window, linerly interplolate the
  // lookahead/lookback station for "soft" prediction window switch
  double lookahead_station = 0.0;
  double lookback_station = 0.0;
  if (std::fabs(linear_v) >= low_speed_bound_) {
    lookahead_station = lookahead_station_high_speed_;
    lookback_station = lookback_station_high_speed_;
  } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
    lookahead_station = lookahead_station_low_speed_;
    lookback_station = lookback_station_low_speed_;
  } else {
    lookahead_station = common::math::lerp(
        lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
    lookback_station = common::math::lerp(
        lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
  }

  // Estimate the heading error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double heading_error_feedback;
  
  auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
      target_point.relative_time +
      lookahead_station /
          (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));
  heading_error_feedback = common::math::NormalizeAngle(
      heading_error + target_point.path_point.theta -
      lookahead_point.path_point.theta);
  
  debug->set_heading_error_feedback(heading_error_feedback);

  // Estimate the lateral error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double lateral_error_feedback;
  
  lateral_error_feedback =
      lateral_error + lookahead_station * std::sin(heading_error);
  
  debug->set_lateral_error_feedback(lateral_error_feedback);

  auto lateral_error_dot = linear_v * std::sin(heading_error);
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
  
  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  
  debug->set_heading_rate(angular_v);

  debug->set_ref_heading_rate(target_point.path_point.kappa *
                              target_point.v);
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

  debug->set_curvature(target_point.path_point.kappa);
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
  //     EDEBUG << "Matrix_b changed due to gear direction";
  //   }
  // }
}

} // namespace control
} // namespace EDrive