/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "ros/ros.h"

#include "common/src/macro.h"
#include "common/src/adapters/proto/adapter_config.pb.h"

namespace EDrive {
namespace common {
namespace adapter{

/// Macro to prepare all the necessary adapter functions when adding a
/// new input/output. For example when you want to listen to
/// car_status message for your module, you can do
/// REGISTER_ADAPTER(CarStatus) write an adapter class called
/// CarStatusAdapter, and call EnableCarStatus(`car_status_topic`,
/// true, `callback`(if there's one)) in AdapterManager.
#define REGISTER_ADAPTER(name)                                                 \
 public:                                                                       \
  static void Enable##name(const std::string &topic_name,                      \
                           const AdapterConfig &config) {                      \
}                                                                              \

/**
 * @class AdapterManager
 *
 * @brief this class hosts all the specific adapters and manages them.
 * It provides APIs for the users to initialize, access and interact
 * with the adapters that they are interested in.
 *
 * \par
 * Each (potentially) useful adapter needs to be registered here with
 * the macro REGISTER_ADAPTER.
 *
 * \par
 * The AdapterManager is a singleton.
 */
class AdapterManager
{
 public:
  /**
   * @brief Initialize the \class AdapterManager singleton with the
   * provided configuration. The configuration is specified by the
   * file path.
   * @param adapter_config_filename the path to the proto file that
   * contains the adapter manager configuration.
   */
  static void Init(const std::string &adapter_config_filename);

  /**
   * @brief Initialize the \class AdapterManager singleton with the
   * provided configuration.
   * @param configs the adapter manager configuration proto.
   */
  // static void Init(const AdapterManagerConfig &configs);

  /**
   * @brief Resets the \class AdapterManager so that it could be
   * re-initiailized.
   */
  static void Reset();

  /**
   * @brief check if the AdapterManager is initialized
   */
  static bool Initialized();

  static void Observe();

    /**
   * @brief create a timer which will call a callback at the specified
   * rate. It takes a class member function, and a bare pointer to the
   * object to call the method on.
   */

  template <class T>
  static ros::Timer CreateTimer(ros::Duration period,
                                void (T::*callback)(const ros::TimerEvent &),
                                T *obj, bool oneshot = false,
                                bool autostart = true) {
    static ros::NodeHandle nh;
    return nh.createTimer(period, callback, obj, oneshot, autostart);
  }

 private:
  /// The node handler of ROS, owned by the \class AdapterManager
  /// singleton.
  std::unique_ptr<ros::NodeHandle> node_handle_;

  /// Observe() callbacks that will be used to to call the Observe()
  /// of enabled adapters.
  std::vector<std::function<void()>> observers_;

  bool initialized_ = false;

  /// The following code registered all the adapters of interest.
  REGISTER_ADAPTER(ControlCommand);
};

} // adapter
} // common
} // EDrive