/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/filters/digital_filter.h"

#include <cmath>

#include "common/src/log.h"

namespace {

const double kDoubleEpsilon = 1.0e-6;

}  // namespace

namespace EDirve {
namespace common {

DigitalFilter::DigitalFilter(const std::vector<double> &denominators,
                             const std::vector<double> &numerators) {
  set_coefficients(denominators, numerators);
}

void DigitalFilter::set_denominators(const std::vector<double> &denominators) {
  denominators_ = denominators;
  y_values_.resize(denominators_.size(), 0.0);
}

void DigitalFilter::set_numerators(const std::vector<double> &numerators) {
  numerators_ = numerators;
  x_values_.resize(numerators_.size(), 0.0);
}

void DigitalFilter::set_coefficients(const std::vector<double> &denominators,
                                     const std::vector<double> &numerators) {
  set_denominators(denominators);
  set_numerators(numerators);
}

void DigitalFilter::set_dead_zone(const double deadzone) {
  dead_zone_ = std::abs(deadzone);
  EINFO("Setting digital filter dead zone = %f", dead_zone_);
}

double DigitalFilter::Filter(const double x_insert) {
  if (denominators_.empty() || numerators_.empty()) {
    EERROR("Empty denominators or numerators");
    return 0.0;
  }

  x_values_.pop_back();
  x_values_.push_front(x_insert);
  const double xside =
      Compute(x_values_, numerators_, 0, numerators_.size() - 1);

  y_values_.pop_back();
  const double yside =
      Compute(y_values_, denominators_, 1, denominators_.size() - 1);

  double y_insert = 0.0;
  if (std::abs(denominators_.front()) > kDoubleEpsilon) {
    y_insert = (xside - yside) / denominators_.front();
  }
  y_values_.push_front(y_insert);

  return UpdateLast(y_insert);
}

double DigitalFilter::UpdateLast(const double input) {
  const double diff = std::abs(input - last_);
  if (diff < dead_zone_) {
    return last_;
  } else {
    last_ = input;
    return input;
  }
}

double DigitalFilter::Compute(const std::deque<double> &values,
                              const std::vector<double> &coefficients,
                              const std::size_t coeff_start,
                              const std::size_t coeff_end) {
  if (coeff_start > coeff_end || coeff_end >= coefficients.size()) {
    EERROR("Invalid inputs.");
    return 0.0;
  }
  if (coeff_end - coeff_start + 1 != values.size()) {
    EERROR("Sizes not match.");
    return 0.0;
  }
  double sum = 0.0;
  int i = coeff_start;
  for (auto &value : values) {
    sum += value * coefficients[i];
    ++i;
  }
  return sum;
}

const std::vector<double> &DigitalFilter::denominators() const {
  return denominators_;
}

const std::vector<double> &DigitalFilter::numerators() const {
  return numerators_;
}

double DigitalFilter::dead_zone() const { return dead_zone_; }

}  // namespace common
}  // namespace EDrive
