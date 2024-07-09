/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <ros/ros.h>
#include <google/protobuf/message.h>

#include "common/adapters/adapter.h"
#include "common/src/log.h"
#include "common/src/macro.h"
#include "common/adapters/proto/adapter_config.pb.h"
#include "common/adapters/message_adapters.h"

namespace EDrive {
namespace common {
namespace adapter {

template<typename T>
using is_protobuf_message = std::is_base_of<google::protobuf::Message, T>;

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
    if(config.message_history_limit() <= 0){                                   \
      ROS_ERROR("    Message history limit must be greater than 0");           \
    }                                                                          \
    Instance()->InternalEnable##name(topic_name, config);                      \
  }                                                                            \
  static name##Adapter *Get##name() {                                          \
    return Instance()->InternalGet##name();                                    \
  }                                                                            \
  static void Publish##name(const name##Adapter::DataType &data) {             \
    Instance()->InternalPublish##name(data);                                   \
  }                                                                            \
  static void Add##name##Callback(name##Adapter::Callback callback) {          \
    CHECK(Instance()->name##_)                                                 \
        << "Initialize adapter before setting callback";                       \
    Instance()->name##_->AddCallback(callback);                                \
  }                                                                            \
  template <class T>                                                           \
  static void Add##name##Callback(                                             \
      void (T::*fp)(const name##Adapter::DataType &data), T *obj) {            \
    Add##name##Callback(std::bind(fp, obj, std::placeholders::_1));            \
  }                                                                            \
  template <class T>                                                           \
  static void Add##name##Callback(                                             \
      void (T::*fp)(const name##Adapter::DataType &data)) {                    \
    Add##name##Callback(fp);                                                   \
  }                                                                            \
  /* Returns false if there's no callback to pop out, true otherwise. */       \
  static bool Pop##name##Callback() {                                          \
    return Instance()->name##_->PopCallback();                                 \
  }                                                                            \
                                                                               \
 private:                                                                      \
  std::unique_ptr<name##Adapter> name##_;                                      \
  ros::Publisher name##publisher_;                                             \
  ros::Subscriber name##subscriber_;                                           \
  AdapterConfig name##config_;                                                 \
                                                                               \
  name##Adapter *InternalGet##name() { return name##_.get(); }                 \
                                                                               \
  template <typename T = name##Adapter::DataType>                              \
  typename std::enable_if<is_protobuf_message<T>::value>::type                 \
  InternalEnable##name(const std::string &topic_name,                          \
                       const AdapterConfig &config) {                          \
    name##_.reset(                                                             \
        new name##Adapter(#name, topic_name, config.message_history_limit())); \
    if (config.mode() != AdapterConfig::PUBLISH_ONLY) {                        \
      ROS_INFO("    registering subscriber: %s", topic_name.c_str());          \
      name##subscriber_ =                                                      \
          node_handle_->subscribe(topic_name, config.message_history_limit(),  \
                                  &name##Adapter::RosCallbackByteArray,        \
                                  name##_.get());                              \
    }                                                                          \
    if (config.mode() != AdapterConfig::RECEIVE_ONLY) {                        \
      ROS_INFO("    registering publisher: %s", topic_name.c_str());           \
      name##publisher_ = node_handle_->advertise<std_msgs::ByteMultiArray>(    \
               topic_name, config.message_history_limit(), config.latch());    \
    }                                                                          \
    observers_.push_back([this]() { name##_->Observe(); });                    \
    name##config_ = config;                                                    \
  }                                                                            \
                                                                               \
  template <typename T = name##Adapter::DataType>                              \
  typename std::enable_if<!is_protobuf_message<T>::value>::type                \
  InternalEnable##name(const std::string &topic_name,                          \
                       const AdapterConfig &config) {                          \
    name##_.reset(                                                             \
        new name##Adapter(#name, topic_name, config.message_history_limit())); \
    if (config.mode() != AdapterConfig::PUBLISH_ONLY) {                        \
      ROS_INFO("    registering subscriber: %s", topic_name.c_str());          \
      name##subscriber_ =                                                      \
          node_handle_->subscribe(topic_name, config.message_history_limit(),  \
                                  &name##Adapter::RosCallback, name##_.get()); \
    }                                                                          \
    if (config.mode() != AdapterConfig::RECEIVE_ONLY) {                        \
      ROS_INFO("    registering publisher: %s", topic_name.c_str());           \
      name##publisher_ = node_handle_->advertise<name##Adapter::DataType>(     \
               topic_name, config.message_history_limit(), config.latch());    \
    }                                                                          \
    observers_.push_back([this]() { name##_->Observe(); });                    \
    name##config_ = config;                                                    \
  }                                                                            \
                                                                               \
  void InternalPublish##name(const name##Adapter::DataType &data) {            \
    name##_->Publish(name##publisher_, data);                                  \
  }


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
  static void Init(const AdapterManagerConfig &configs);

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
    return Instance()->node_handle_->createTimer(period, callback, obj,
                                                   oneshot, autostart);
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
  // REGISTER_ADAPTER(Viewer);
  REGISTER_ADAPTER(Test);
  REGISTER_ADAPTER(RoutingRequest);
  REGISTER_ADAPTER(RoutingResponse);
  REGISTER_ADAPTER(RelativeMap);
  REGISTER_ADAPTER(Navigation);
  REGISTER_ADAPTER(Vehicle);
  REGISTER_ADAPTER(Planning);
  REGISTER_ADAPTER(ControlCommand);
  REGISTER_ADAPTER(CarlaObjects);
  REGISTER_ADAPTER(ViewerObjects);
  REGISTER_ADAPTER(Perception);
  REGISTER_ADAPTER(Localization);
  REGISTER_ADAPTER(ViewerPath);
  REGISTER_ADAPTER(RoadMarkings);
  REGISTER_ADAPTER(RoadMarker);
  REGISTER_ADAPTER(VehicleLocation);
  REGISTER_ADAPTER(VectorMap);
  REGISTER_ADAPTER(CloudPointMap);
  REGISTER_ADAPTER(FixedPath);
  REGISTER_ADAPTER(VehicleStatus);
  REGISTER_ADAPTER(MarkerDebugPoint);
  DECLARE_SINGLETON(AdapterManager);
};

} // adapter
} // common
} // EDrive