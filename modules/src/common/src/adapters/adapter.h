/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <mutex>
#include "std_msgs/String.h"

/**
 * @namespace EDrive::common::adapter
 * @brief EDrive::common::adapter
 */
namespace EDrive {
namespace common {
namespace adapter {

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
 * abstraction for Apollo modules to interact with various I/O (e.g.
 * ROS). The adapter will also store history data, so that other
 * Apollo modules can get access to both the current and the past data
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
template <typename D>
class Adapter : public AdapterBase {
 public:
  /// The user can use Adapter::DataType to get the type of the
  /// underlying data.
  typedef D DataType;
  typedef boost::shared_ptr<D const> DataPtr;

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
  Adapter(const std::string& adapter_name, const std::string& topic_name,
          size_t message_num)
      : topic_name_(topic_name),
        message_num_(message_num) {
  }

  /**
   * @brief copy the data_queue_ into the observing queue to create a
   * view of data up to the call time for the user.
   */
  void Observe() override {
    std::lock_guard<std::mutex> lock(mutex_);
    observed_queue_ = data_queue_;
  }

  /**
   * @brief returns the most recent message in the observing queue.
   *
   * \note
   * Please call Empty() to make sure that there is data in the
   * queue before calling GetLatestObserved().
   */
  const D& GetLatestObserved() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(observed_queue_.empty()) {
      ROS_ERROR("The view of data queue is empty. No data is received yet or you");
    }

    return *observed_queue_.front();
  }

 private:

   /**
   * @brief returns the topic name that this adapter listens to.
   */
  const std::string& topic_name() const override { return topic_name_; }

  /**
   * @brief the ROS callback that will be invoked whenever a new
   * message is received.
   * @param message the newly received message.
   */
  void RosCallback(DataPtr message) {
    last_receive_time_ = ros::Time::now();
    EnqueueData(message);
  }

    /**
   * @brief push the shared-pointer-guarded data to the data queue of
   * the adapter.
   */
  void EnqueueData(DataPtr data) {
    // Don't try to copy data and enqueue if the message_num is 0
    if (message_num_ == 0) {
      return;
    }
    
    // Lock the queue.
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_queue_.size() + 1 > message_num_) {
      data_queue_.pop_back();
    }
    data_queue_.push_front(data);
  }

  /// The topic name that the adapter listens to.
  std::string topic_name_;

  /// The maximum size of data_queue_ and observed_queue_
  size_t message_num_ = 0;

  /// The mutex guarding data_queue_ and observed_queue_
  mutable std::mutex mutex_;

  /// The received data. Its size is no more than message_num_
  std::list<DataPtr> data_queue_;

  /// It is the snapshot of the data queue. The snapshot is taken when
  /// Observe() is called.
  std::list<DataPtr> observed_queue_;
  
  ros::Time last_receive_time_;
  
  friend class AdapterManager;
};

} // namespace adapter
} // namespace common
} // namespace EDrive