/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
/**
 * @file vehicle_config_helper.h
 *
 * @brief Declaration of the class VehicleConfigHelper.
 */
#pragma once

#include <string>

#include "common/configs/proto/vehicle_config.pb.h"
#include "common/src/macro.h"

/**
 * @namespace EDrive::common
 * @brief EDrive::common
 */
namespace EDrive {
namespace common {

/**
 * @class VehicleConfigHelper
 *
 * @Brief This is a helper class that can load vehicle configurations. The
 * vehicle configurations are
 * defined common/configs/proto/vehicle_config.proto
 */
class VehicleConfigHelper {
 public:
  /**
   * @brief Initialize vehicle configurations with default configuration file
   * pointed by gflags FLAGS_vehicle_config_path. The code will crash if
   * FLAGS_vehicle_config_path does not exist or it points to a file with invalid
   * format.
   */
  static void Init();

  /**
   * @brief Initialize vehicle configurations with \p config
   * @param config A VehicleConfig class instance. The VehicleConfig class is
   * defined by common/configs/proto/vehicle_config.proto.
   */
  static void Init(const VehicleConfig &config);

  /**
   * @brief Initialize vehicle configurations with \p config_file.
   * The code will crash if \p config_file does not exist or \p config_file has
   * invalid format.
   * @param config_file The configuration file path. The format of the file is
   * defined by protobuf file
   * common/configs/proto/vehicle_config.proto.
   */
  static void Init(const std::string &config_file);

  /**
   * @brief Get the current vehicle configuration.
   * @return the current VehicleConfig instance reference.
   */
  static const VehicleConfig &GetConfig();

  /**
   * @brief Get the safe turning radius when the vehicle is turning with
   * maximum steering angle.
   *
   * The calculation is described by the following figure.
   *  <pre>
   *
   *
   *    front of car
   * A +----------+ B
   *   |          |
   *   /          / turn with maximum steering angle
   *   |          |
   *   |          |
   *   |          |
   *   |    X     |                                       O
   *   |<-->.<----|-------------------------------------->* (turn center)
   *   |          |   VehicleParam.min_turn_radius()
   *   |          |
   * D +----------+ C
   *    back of car
   *
   *  </pre>
   *
   *  In the above figure, The four corner points of the vehicle is A, B, C, and
   * D. XO is VehicleParam.min_turn_radius(), X to AD is left_edge_to_center,
   * X to AB is VehicleParam.front_edge_to_center(). Then
   *     AO = sqrt((XO +  left_edge_to_center) ^2 + front_edge_to_center^2).
   * @return AO in the above figure, which is the maximum turn radius when the
   * vehicle turns with maximum steering angle
   */

  static double MinSafeTurnRadius();

  static std::string root_path;
  static std::string conf_file;

 private:
  static VehicleConfig vehicle_config_;
  static bool is_init_;
  DECLARE_SINGLETON(VehicleConfigHelper);
};

}  // namespace common
}  // namespace EDrive
