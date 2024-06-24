/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 * @brief Defines the MeanFilter class.
 */

#pragma once

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

/**
 * @namespace EDrive::common
 * @brief The EDrive::common namespace contains the code of the common module.
 */
namespace EDrive {
namespace common {

/**
 * @class MeanFilter
 * @brief The MeanFilter class is used to smoothen a series of noisy numbers,
 * such as sensor data or the output of a function that we wish to be smoother.
 *
 * This is achieved by keeping track of the last k measurements
 * (where k is the window size), and returning the average of all but the
 * minimum and maximum measurements, which are likely to be outliers.
 */
class MeanFilter {
 public:
  /**
   * @brief Initializes a MeanFilter with a given window size.
   * @param window_size The window size of the MeanFilter.
   * Older measurements are discarded.
   */
  explicit MeanFilter(const std::uint_fast8_t window_size);
  /**
   * @brief Default constructor; defers initialization.
   */
  MeanFilter() = default;
  /**
   * @brief Default destructor.
   */
  ~MeanFilter() = default;
  /**
   * @brief Processes a new measurement in amortized constant time.
   * @param measurement The measurement to be processed by the filter.
   */
  double Update(const double measurement);

 private:
  void RemoveEarliest();
  void Insert(const double value);
  double GetMin() const;
  double GetMax() const;
  bool ShouldPopOldestCandidate(const std::uint_fast8_t old_time) const;
  std::uint_fast8_t window_size_ = 0;
  double sum_ = 0.0;
  std::uint_fast8_t time_ = 0;
  // front = earliest
  std::deque<double> values_;
  // front = min
  std::deque<std::pair<std::uint_fast8_t, double>> min_candidates_;
  // front = max
  std::deque<std::pair<std::uint_fast8_t, double>> max_candidates_;
  bool initialized_ = false;
};

}  // namespace common
}  // namespace EDrive
