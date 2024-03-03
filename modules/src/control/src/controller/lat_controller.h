/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "control/src/controller/controller.h"

#include "control/src/common/trajectory_analyzer.h"

#include <eigen3/Eigen/Core>

namespace EDrive {
namespace control {

using EDrive::Result_state;

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
  Result_state Init(const ControlConf *control_conf) override;

  /**
   * @brief compute steering target based on current vehicle status
   *        and target trajectory
   * @param localization vehicle location
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  Result_state ComputeControlCommand(
      const ::planning::ADCTrajectory *trajectory,
      const nav_msgs::Odometry *localization,
      ::control::CarlaEgoVehicleControl *control_command) override;

  /**
   * @brief reset Lateral Controller
   * @return Status reset status
   */
  Result_state Reset() override;

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

 protected:
  void UpdateStateAnalyticalMatching(SimpleLateralDebug *debug);

  bool LoadControlConf(const ControlConf *control_conf);

  void ComputeLateralErrors(const TrajectoryAnalyzer *trajectory_analyzer);

  void UpdateMatrix();

  void UpdateMatrixCompound();

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
  double steer_transmission_ratio_ = 0.0;
  // the maximum turn of steer
  double steer_single_direction_max_degree_ = 0.0;

  // number of control cycles look ahead (preview controller)
  int preview_window_ = 0;
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

};

} // namespace control
} // namespace EDrive