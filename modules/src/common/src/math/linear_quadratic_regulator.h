/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 * @brief Solver for discrete-time linear quadratic problem.
 */

#pragma once

#include <eigen3/Eigen/Core>

/**
 * @namespace EDrive::common::math
 * @brief EDrive::common::math
 */
namespace EDrive {
namespace common {
namespace math {

/**
 * @brief Solver for discrete-time linear quadratic problem.
 * @param A The system dynamic matrix
 * @param B The control matrix
 * @param Q The cost matrix for system state
 * @param R The cost matrix for control output
 * @param tolerance The numerical tolerance for solving
 *        Algebraic Riccati equation (ARE)
 * @param max_num_iteration The maximum iterations for solving ARE
 * @param ptr_K The feedback control matrix (pointer)
 */
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const double tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K);

}  // namespace math
}  // namespace common
}  // namespace EDrive
