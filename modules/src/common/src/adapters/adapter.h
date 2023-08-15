/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

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

 private:
  /**
   * @brief the ROS callback that will be invoked whenever a new
   * message is received.
   * @param message the newly received message.
   */
  void RosCallback(DataPtr message) {

  }
  
  friend class AdapterManager;
};

} // namespace adapter
} // namespace common
} // namespace EDrive