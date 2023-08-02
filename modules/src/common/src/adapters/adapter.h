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

    /**
   * @brief Create a view of data up to the call time for the user.
   */
  virtual void Observe() = 0;

  /**
   * @brief returns TRUE if the observing queue is empty.
   */
  virtual bool Empty() const = 0;

  /**
   * @brief returns TRUE if the adapter has received any message.
   */
  virtual bool HasReceived() const = 0;

  /**
   * @brief Gets message delay.
   */
  virtual double GetDelaySec() const = 0;

  /**
   * @brief Clear the data received so far.
   */
  virtual void ClearData() = 0;

  /**
   * @brief Dumps the latest received data to file.
   */
  virtual bool DumpLatestMessage() = 0;
};

} // namespace adapter
} // namespace common
} // namespace EDrive