/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <boost/shared_ptr.hpp>
#include <ros/time.h>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <functional>
#include <google/protobuf/message.h>

#include <type_traits>
#include <utility>

#include <mutex>
#include "std_msgs/String.h"
#include <std_msgs/ByteMultiArray.h>

/**
 * @namespace EDrive::common::adapter
 * @brief EDrive::common::adapter
 */
namespace EDrive {
namespace common {
namespace adapter {

template<typename T>
class Container {
public:
    using DataPtr = boost::shared_ptr<const T>;

    Container(size_t message_num) : message_num_(message_num) {}

    void EnqueueData(const DataPtr& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (data_queue_.size() >= message_num_) {
            data_queue_.pop_back();
        }
        data_queue_.push_front(data);
    }

    void Observe() {
        std::lock_guard<std::mutex> lock(mutex_);
        observed_queue_ = data_queue_;
    }

    DataPtr GetLatestObserved() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (observed_queue_.empty()) {
            ROS_ERROR("The view of data queue is empty. No data is received yet.");
        }
        return observed_queue_.front();
    }

private:
    size_t message_num_;
    mutable std::mutex mutex_;
    std::list<DataPtr> data_queue_;
    std::list<DataPtr> observed_queue_;
};

/**
 * @class AdapterBase
 * @brief Base interface of all concrete adapters.
 */
class AdapterBase {
 public:
  virtual ~AdapterBase() = default;

  /**
   * @brief returns the topic name that this adapter listens to.
   */
  virtual const std::string& topic_name() const = 0;

  /**
   * @brief Create a view of data up to the call time for the user.
   */
  virtual void Observe() = 0;

};

/**
 * @class Adapter
 * @brief this class serves as the interface and a layer of
 * abstraction for EDrive modules to interact with various I/O (e.g.
 * ROS). The adapter will also store history data, so that other
 * EDrive modules can get access to both the current and the past data
 * without having to handle communication protocols directly.
 *
 * \par
 * Each \class Adapter instance only works with one single topic and
 * its corresponding data type.
 *
 * \par
 * Under the hood, a queue is used to store the current and historical
 * messages. In most cases, the underlying data type is a proto, though
 * this is not necessary.
 *
 * \note
 * Adapter::Observe() is thread-safe, but calling it from
 * multiple threads may introduce unexpected behavior. Adapter is
 * thread-safe w.r.t. data access and update.
 */
template<typename T>
using is_protobuf_message = std::is_base_of<google::protobuf::Message, T>;

template <typename D>
class Adapter : public AdapterBase {
 public:
  /// The user can use Adapter::DataType to get the type of the
  /// underlying data.
  typedef D DataType;
  using DataPtr = boost::shared_ptr<D const>;
  typedef std::vector<uint8_t> ByteString;

  typedef typename std::list<DataPtr>::const_iterator Iterator;
  typedef typename std::function<void(const D&)> Callback;

  /**
   * @brief Construct the \class Adapter object.
   * @param adapter_name the name of the adapter. It is used to log
   * error messages when something bad happens, to help people get an
   * idea which adapter goes wrong.
   * @param topic_name the topic that the adapter listens to.
   * @param message_num the number of historical messages that the
   * adapter stores. Older messages will be removed upon calls to
   * Adapter::RosCallback().
   */
  Adapter(const std::string& adapter_name, const std::string& topic_name, size_t message_num)
      : topic_name_(topic_name),
        message_num_(message_num),
        proto_container_(message_num),
        ros_container_(message_num),
        use_proto_container_(IsProtocolBufferType()) {}
  
  bool IsProtocolBufferType() const {
    return is_protobuf_message<D>::value;
  }

  /**
   * @brief the ROS callback that will be invoked whenever a new
   * message is received.
   * @param message the newly received message.
   */
  void RosCallback(DataPtr message) {
    last_receive_time_ = ros::Time::now();
    if (use_proto_container_) {
        SerializeAndEnqueue(message);
    } else {
        ros_container_.EnqueueData(message);
    }
  }

  void RosCallbackByteArray(const std_msgs::ByteMultiArray::ConstPtr& msg) {
    last_receive_time_ = ros::Time::now();
    auto byteData = boost::make_shared<ByteString>(msg->data.begin(), msg->data.end());
    proto_container_.EnqueueData(byteData);
  }

  /**
   * @brief copy the data_queue_ into the observing queue to create a
   * view of data up to the call time for the user.
   */
  void Observe() override {
    if (use_proto_container_) {
        proto_container_.Observe();
    } else {
        ros_container_.Observe();
    }
  }

  /**
   * @brief returns the most recent message in the observing queue.
   *
   * \note
   * Please call Empty() to make sure that there is data in the
   * queue before calling GetLatestObserved().
   */
  const D& GetLatestObserved() const {
    if (use_proto_container_) {
        // Get the latest ByteString from proto_container_ and deserialize it.
        auto byteStringData = proto_container_.GetLatestObserved();
        auto mutableByteStringData = boost::const_pointer_cast<ByteString>(byteStringData);
        auto deserialized_message = DeserializeFromByteString(mutableByteStringData);
        last_deserialized_message_ = deserialized_message;  // Cache the deserialized message
        return *deserialized_message;
    } else {
        return *ros_container_.GetLatestObserved();
    }
  }

  bool use_proto_container_;

  template <typename T = D>
  typename std::enable_if<is_protobuf_message<T>::value>::type
  Publish(ros::Publisher& publisher, const DataType& data) {
    std_msgs::ByteMultiArray byte_array_msg;
    byte_array_msg.data.resize(data.ByteSizeLong());
    data.SerializeToArray(byte_array_msg.data.data(), byte_array_msg.data.size());
    publisher.publish(byte_array_msg);
  }

  template <typename T = D>
  typename std::enable_if<!is_protobuf_message<T>::value>::type
  Publish(ros::Publisher& publisher, const DataType& data) {
    publisher.publish(data);
  }

 private:
  template<typename T = D>
  typename std::enable_if<is_protobuf_message<T>::value>::type
  SerializeAndEnqueue(DataPtr message) {
    ByteString serializedData(message->ByteSizeLong());
    message->SerializeToArray(serializedData.data(), serializedData.size());
    auto bytePtr = boost::make_shared<ByteString>(serializedData);
    proto_container_.EnqueueData(bytePtr);
  }

  template<typename T = D>
  typename std::enable_if<!is_protobuf_message<T>::value>::type
  SerializeAndEnqueue(DataPtr) {
    // No-op for non-Protocol Buffers types
  }

  template<typename T = D>
  typename std::enable_if<is_protobuf_message<T>::value, DataPtr>::type
  DeserializeFromByteString(const boost::shared_ptr<ByteString>& byteString) const {
    auto message = boost::make_shared<D>();
    if (!message->ParseFromArray(byteString->data(), byteString->size())) {
        throw std::runtime_error("Failed to parse byte string to message");
    }
    return message;
  }

  template<typename T = D>
  typename std::enable_if<!is_protobuf_message<T>::value, DataPtr>::type
  DeserializeFromByteString(const boost::shared_ptr<ByteString>&) const {
    throw std::runtime_error("DeserializeFromByteString called on non-protocol buffer type");
  }

   /**
   * @brief returns the topic name that this adapter listens to.
   */
  const std::string& topic_name() const override { return topic_name_; }

  /// The topic name that the adapter listens to.
  std::string topic_name_;

  /// The maximum size of data_queue_ and observed_queue_
  size_t message_num_ = 0;

  /// The received data. Its size is no more than message_num_
  std::list<DataPtr> data_queue_;

  /// It is the snapshot of the data queue. The snapshot is taken when
  /// Observe() is called.
  std::list<DataPtr> observed_queue_;
  
  ros::Time last_receive_time_;

  Container<ByteString> proto_container_;

  Container<D> ros_container_;

  mutable DataPtr last_deserialized_message_;
  
  friend class AdapterManager;
};

} // namespace adapter
} // namespace common
} // namespace EDrive