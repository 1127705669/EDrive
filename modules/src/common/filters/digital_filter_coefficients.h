/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file digital_filter_coefficients.h
 * @brief Functions to generate coefficients for digital filter.
 */

#pragma once

#include <vector>

/**
 * @namespace EDrive::common
 * @brief EDrive::common
 */
namespace EDrive {
namespace common {

/**
 * @brief Get low-pass coefficients for digital filter.
 * @param ts Time interval between signals.
 * @param cutoff_freq Cutoff of frequency to filter high-frequency signals out.
 * @param denominators Denominator coefficients for digital filter.
 * @param numerators Numerator coefficients for digital filter.
 */
void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *numerators);

}  // namespace common
}  // namespace EDrive
