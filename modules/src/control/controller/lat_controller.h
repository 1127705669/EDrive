/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "common/configs/proto/vehicle_config.pb.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/filters/mean_filter.h"
#include "control/common/interpolation_1d.h"
#include "control/common/trajectory_analyzer.h"
#include "control/controller/controller.h"


namespace EDrive {
namespace control {

/**
 * @class LatController
 *
 * @brief Lateral controller, to steer.
 */
class LatController : public Controller {
 public:
  /**
   * @brief constructor
   */
  LatController();

  /**
   * @brief destructor
   */
  virtual ~LatController();

  /**
   * @brief initialize Lateral Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  common::Result_state Init(const ControlConf *control_conf) override;

  /**
   * @brief compute steering target based on current vehicle status
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
      ControlCommand *cmd) override;

  /**
   * @brief reset Lateral Controller
   * @return Status reset status
   */
  common::Result_state Reset() override;

  /**
   * @brief stop Lateral controller
   */
  void Stop() override;

  /**
   * @brief Lateral controller name
   * @return string controller name in string
   */
  std::string Name() const override;

 private:
  std::string name_;

  bool controller_initialized_ = false;
  const ControlConf *control_conf_ = nullptr;

  // vehicle parameter
  common::VehicleParam vehicle_param_;

 protected:
  void UpdateStateAnalyticalMatching(SimpleLateralDebug *debug);

  bool LoadControlConf(const ControlConf *control_conf);

  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug);
  
  // logic for reverse driving mode
  void UpdateDrivingOrientation();

  void UpdateMatrix();

  void UpdateMatrixCompound();

  void InitializeFilters(const ControlConf *control_conf);
  void LoadLatGainScheduler(const LatControllerConf &lat_controller_conf);
  void LogInitParameters();

  // a proxy to analyze the planning trajectory
  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;

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

  // limit steering to maximum theoretical lateral acceleration
  double max_lat_acc_ = 0.0;

  // number of control cycles look ahead (preview controller)
  int preview_window_ = 0;

  // longitudial length for look-ahead lateral error estimation during forward
  // driving and look-back lateral error estimation during backward driving
  // (look-ahead controller)
  double lookahead_station_low_speed_ = 0.0;
  double lookback_station_low_speed_ = 0.0;
  double lookahead_station_high_speed_ = 0.0;
  double lookback_station_high_speed_ = 0.0;

  // number of states without previews, includes
  // lateral error, lateral error rate, heading error, heading error rate
  const int basic_state_size_ = 4;
  // vehicle state matrix
  Eigen::MatrixXd matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd matrix_ad_;
  // vehicle state matrix compound; related to preview
  Eigen::MatrixXd matrix_adc_;
  // control matrix
  Eigen::MatrixXd matrix_b_;
  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_bd_;
  // control matrix compound
  Eigen::MatrixXd matrix_bdc_;
  // gain matrix
  Eigen::MatrixXd matrix_k_;
  // control authority weighting matrix
  Eigen::MatrixXd matrix_r_;
  // state weighting matrix
  Eigen::MatrixXd matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd matrix_q_updated_;
  // vehicle state matrix coefficients
  Eigen::MatrixXd matrix_a_coeff_;
  // 4 by 1 matrix; state matrix
  Eigen::MatrixXd matrix_state_;

  // parameters for lqr solver; number of iterations
  int lqr_max_iteration_ = 0;
  // parameters for lqr solver; threshold for computation
  double lqr_eps_ = 0.0;

  common::DigitalFilter digital_filter_;

  std::unique_ptr<Interpolation1D> lat_err_interpolation_;

  std::unique_ptr<Interpolation1D> heading_err_interpolation_;

  // MeanFilter heading_rate_filter_;
  common::MeanFilter lateral_error_filter_;
  common::MeanFilter heading_error_filter_;

  double query_relative_time_;

  double minimum_speed_protection_ = 0.1;

  double low_speed_bound_ = 0.0;

  double low_speed_window_ = 0.0;

  double driving_orientation_ = 0.0;
};

} // namespace control
} // namespace EDrive