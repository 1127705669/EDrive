/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 * @brief Defines the MPCController class.
 */

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "common/configs/proto/vehicle_config.pb.h"

#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/filters/mean_filter.h"

#include "control/common/interpolation_1d.h"
#include "control/common/interpolation_2d.h"
#include "control/controller/controller.h"
#include "control/common/trajectory_analyzer.h"

/**
 * @namespace EDrive::control
 * @brief EDrive::control
 */
namespace EDrive {
namespace control {

/**
 * @class MPCController
 *
 * @brief MPCController, combined lateral and logitudinal controllers
 */
class MPCController : public Controller {
 public:
  /**
   * @brief constructor
   */
  MPCController();

  /**
   * @brief destructor
   */
  virtual ~MPCController();

  /**
   * @brief initialize MPC Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  common::Result_state Init(const ControlConf *control_conf) override;

     /**
   * @brief compute control command based on current vehicle status
   *        and target trajectory
   * @param localization vehicle location
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  common::Result_state ComputeControlCommand(
      const ::planning::ADCTrajectory *trajectory,
      const nav_msgs::Odometry *localization,
      ControlCommand *control_command) override;

  /**
   * @brief reset MPC Controller
   * @return Status reset status
   */
  common::Result_state Reset() override;

  /**
   * @brief stop MPC controller
   */
  void Stop() override;

  /**
   * @brief MPC controller name
   * @return string controller name in string
   */
  std::string Name() const override;

 protected:
  void UpdateState(SimpleMPCDebug *debug);

  void UpdateMatrix(SimpleMPCDebug *debug);

  void FeedforwardUpdate(SimpleMPCDebug *debug);

  double GetLateralError(
      const common::math::Vec2d &point,
      ::common::TrajectoryPoint *trajectory_point) const;

  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleMPCDebug *debug);

  void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory,
                                 SimpleMPCDebug *debug);
                                 
  bool LoadControlConf(const ControlConf *control_conf);
  void InitializeFilters(const ControlConf *control_conf);
  void LogInitParameters();

  void CloseLogFile();

  // vehicle parameter
  common::VehicleParam vehicle_param_;

  // a proxy to analyze the planning trajectory
  TrajectoryAnalyzer trajectory_analyzer_;

  void LoadControlCalibrationTable(
      const MPCControllerConf &mpc_controller_conf);

  void LoadMPCGainScheduler(const MPCControllerConf &mpc_controller_conf);

  std::unique_ptr<Interpolation2D> control_interpolation_;

  // the following parameters are vehicle physics related.
  // control time interval
  double ts_ = 0.0;
  // corner stiffness; front
  double cf_ = 0.0;
  // corner stiffness; rear
  double cr_ = 0.0;
  // distance between front and rear wheel center
  double wheelbase_ = 0.0;
  // mass of the vehicle
  double mass_ = 0.0;
  // distance from front wheel center to COM
  double lf_ = 0.0;
  // distance from rear wheel center to COM
  double lr_ = 0.0;
  // rotational inertia
  double iz_ = 0.0;
  // the ratio between the turn of the steering wheel and the turn of the wheels
  double steer_ratio_ = 0.0;
  // the maximum turn of steer
  double steer_single_direction_max_degree_ = 0.0;
  // the maximum turn of vehicle wheel
  double wheel_single_direction_max_degree_ = 0.0;

  // limit steering to maximum theoretical lateral acceleration
  double max_lat_acc_ = 0.0;

  // number of states, includes
  // lateral error, lateral error rate, heading error, heading error rate,
  // station error, velocity error,
  const int basic_state_size_ = 6;

  const int controls_ = 2;

  const int horizon_ = 10;

  // vehicle state matrix
  Eigen::MatrixXd matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd matrix_ad_;

  // control matrix
  Eigen::MatrixXd matrix_b_;
  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_bd_;

  // offset matrix
  Eigen::MatrixXd matrix_c_;
  // offset matrix (discrete-time)
  Eigen::MatrixXd matrix_cd_;

  // gain matrix
  Eigen::MatrixXd matrix_k_;
  // control authority weighting matrix
  Eigen::MatrixXd matrix_r_;
  // updated control authority weighting matrix
  Eigen::MatrixXd matrix_r_updated_;
  // state weighting matrix
  Eigen::MatrixXd matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd matrix_q_updated_;
  // vehicle state matrix coefficients
  Eigen::MatrixXd matrix_a_coeff_;
  // 4 by 1 matrix; state matrix
  Eigen::MatrixXd matrix_state_;

  // heading error of last control cycle
  double previous_heading_error_ = 0.0;
  // lateral distance to reference trajectory of last control cycle
  double previous_lateral_error_ = 0.0;

  // lateral dynamic variables for computing the differential valute to
  // estimate acceleration and jerk
  double previous_lateral_acceleration_ = 0.0;

  double previous_heading_rate_ = 0.0;
  double previous_ref_heading_rate_ = 0.0;

  double previous_heading_acceleration_ = 0.0;
  double previous_ref_heading_acceleration_ = 0.0;

  // longitudinal dynamic variables for computing the differential valute to
  // estimate acceleration and jerk
  double previous_acceleration_ = 0.0;
  double previous_acceleration_reference_ = 0.0;

  // parameters for mpc solver; number of iterations
  int mpc_max_iteration_ = 0;
  // parameters for mpc solver; threshold for computation
  double mpc_eps_ = 0.0;

  common::DigitalFilter digital_filter_;

  std::unique_ptr<Interpolation1D> lat_err_interpolation_;

  std::unique_ptr<Interpolation1D> heading_err_interpolation_;

  std::unique_ptr<Interpolation1D> feedforwardterm_interpolation_;

  std::unique_ptr<Interpolation1D> steer_weight_interpolation_;

  // for logging purpose
  std::ofstream mpc_log_file_;

  const std::string name_;

  double standstill_acceleration_ = 0.0;

  double throttle_deadzone_ = 0.0;

  double brake_deadzone_ = 0.0;

  double steer_angle_feedforwardterm_ = 0.0;

  double steer_angle_feedforwardterm_updated_ = 0.0;

  double max_acceleration_ = 0.0;

  double max_deceleration_ = 0.0;

  // MeanFilter heading_error_filter_;
  common::MeanFilter lateral_error_filter_;

  // MeanFilter lateral_error_filter;
  common::MeanFilter heading_error_filter_;

  double minimum_speed_protection_ = 0.1;
};

}  // namespace control
}  // namespace EDrive