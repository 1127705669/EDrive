/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/mpc_solver.h"

#include <algorithm>
#include <memory>

#include "common/math/qp_solver/active_set_qp_solver.h"
#include "common/math/qp_solver/qp_solver.h"

namespace EDrive {
namespace common {
namespace math {

// using Matrix = Eigen::MatrixXd;

// // discrete linear predictive control solver, with control format
// // x(i + 1) = A * x(i) + B * u (i) + C
// bool SolveLinearMPC(const Matrix &matrix_a, const Matrix &matrix_b,
//                     const Matrix &matrix_c, const Matrix &matrix_q,
//                     const Matrix &matrix_r, const Matrix &matrix_lower,
//                     const Matrix &matrix_upper,
//                     const Matrix &matrix_initial_state,
//                     const std::vector<Matrix> &reference, const double eps,
//                     const int max_iter, std::vector<Matrix> *control) {
//   if (matrix_a.rows() != matrix_a.cols() ||
//       matrix_b.rows() != matrix_a.rows() ||
//       matrix_lower.rows() != matrix_upper.rows()) {
//     // EERROR << "One or more matrices have incompatible dimensions. Aborting.";
//     return false;
//   }

//   unsigned int horizon = reference.size();

//   // Update augment reference matrix_t
//   Matrix matrix_t = Matrix::Zero(matrix_b.rows() * horizon, 1);
//   for (unsigned int j = 0; j < horizon; ++j) {
//     matrix_t.block(j * reference[0].size(), 0, reference[0].size(), 1) =
//         reference[j];
//   }

//   // Update augment control matrix_v
//   Matrix matrix_v = Matrix::Zero((*control)[0].rows() * horizon, 1);
//   for (unsigned int j = 0; j < horizon; ++j) {
//     matrix_v.block(j * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
//         (*control)[j];
//   }

//   std::vector<Matrix> matrix_a_power(horizon);
//   matrix_a_power[0] = matrix_a;
//   for (unsigned int i = 1; i < matrix_a_power.size(); ++i) {
//     matrix_a_power[i] = matrix_a * matrix_a_power[i - 1];
//   }

//   Matrix matrix_k =
//       Matrix::Zero(matrix_b.rows() * horizon, matrix_b.cols() * horizon);
//   matrix_k.block(0, 0, matrix_b.rows(), matrix_b.cols()) = matrix_b;
//   for (unsigned int r = 1; r < horizon; ++r) {
//     for (unsigned int c = 0; c < r; ++c) {
//       matrix_k.block(r * matrix_b.rows(), c * matrix_b.cols(), matrix_b.rows(),
//                      matrix_b.cols()) = matrix_a_power[r - c - 1] * matrix_b;
//     }
//     matrix_k.block(r * matrix_b.rows(), r * matrix_b.cols(), matrix_b.rows(),
//                    matrix_b.cols()) = matrix_b;
//   }
//   // Initialize matrix_k, matrix_m, matrix_t and matrix_v, matrix_qq, matrix_rr,
//   // vector of matrix A power
//   Matrix matrix_m = Matrix::Zero(matrix_b.rows() * horizon, 1);
//   Matrix matrix_qq = Matrix::Zero(matrix_k.rows(), matrix_k.rows());
//   Matrix matrix_rr = Matrix::Zero(matrix_k.cols(), matrix_k.cols());
//   Matrix matrix_ll = Matrix::Zero(horizon * matrix_lower.rows(), 1);
//   Matrix matrix_uu = Matrix::Zero(horizon * matrix_upper.rows(), 1);
//   Matrix matrix_cc = Matrix::Zero(horizon * matrix_c.rows(), 1);
//   Matrix matrix_aa = Matrix::Zero(horizon * matrix_a.rows(), matrix_a.cols());
//   matrix_aa.block(0, 0, matrix_a.rows(), matrix_a.cols()) = matrix_a;

//   for (unsigned int i = 1; i < horizon; ++i) {
//     matrix_aa.block(i * matrix_a.rows(), 0, matrix_a.rows(), matrix_a.cols()) =
//         matrix_a * matrix_aa.block((i - 1) * matrix_a.rows(), 0,
//                                    matrix_a.rows(), matrix_a.cols());
//   }

//   // Compute matrix_m
//   matrix_m.block(0, 0, matrix_a.rows(), 1) = matrix_a * matrix_initial_state;
//   for (unsigned int i = 1; i < horizon; ++i) {
//     matrix_m.block(i * matrix_a.rows(), 0, matrix_a.rows(), 1) =
//         matrix_a *
//         matrix_m.block((i - 1) * matrix_a.rows(), 0, matrix_a.rows(), 1);
//   }

//   // Compute matrix_ll, matrix_uu, matrix_qq, matrix_rr
//   for (unsigned int i = 0; i < horizon; ++i) {
//     matrix_ll.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
//         matrix_lower;
//     matrix_uu.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
//         matrix_upper;
//     matrix_qq.block(i * matrix_q.rows(), i * matrix_q.rows(), matrix_q.rows(),
//                     matrix_q.rows()) = matrix_q;
//     matrix_rr.block(i * matrix_r.rows(), i * matrix_r.rows(), matrix_r.cols(),
//                     matrix_r.cols()) = matrix_r;
//   }

//   matrix_cc.block(0, 0, matrix_c.rows(), 1) = matrix_c;
//   for (unsigned int i = 1; i < horizon; ++i) {
//     matrix_cc.block(i * matrix_c.rows(), 0, matrix_c.rows(), 1) =
//         matrix_cc.block((i - 1) * matrix_c.rows(), 0, matrix_c.rows(), 1) +
//         matrix_aa.block((i - 1) * matrix_a.rows(), 0, matrix_a.rows(),
//                         matrix_a.cols()) *
//             matrix_c;
//   }

//   // Update matrix_m1, matrix_m2, convert MPC problem to QP problem done
//   Matrix matrix_m1 = matrix_k.transpose() * matrix_qq * matrix_k + matrix_rr;
//   Matrix matrix_m2 =
//       matrix_k.transpose() * matrix_qq * (matrix_m + matrix_cc - matrix_t);

//   // Format in qp_solver
//   /**
//    * *           min_x  : q(x) = 0.5 * x^T * Q * x  + x^T c
//    * *           with respect to:  A * x = b (equality constraint)
//    * *                             C * x >= d (inequality constraint)
//    * **/

//   // TODO(QiL) : change qp solver to box constraint or substitute QPOASES
//   // Method 1: QPOASES
//   Matrix matrix_inequality_constrain_ll =
//       Matrix::Identity(matrix_ll.rows(), matrix_ll.rows());
//   Matrix matrix_inequality_constrain_uu =
//       Matrix::Identity(matrix_uu.rows(), matrix_uu.rows());
//   Matrix matrix_inequality_constrain =
//       Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.rows());
//   matrix_inequality_constrain << matrix_inequality_constrain_ll,
//       -matrix_inequality_constrain_uu;
//   Matrix matrix_inequality_boundary =
//       Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());
//   matrix_inequality_boundary << matrix_ll, -matrix_uu;
//   Matrix matrix_equality_constrain =
//       Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.rows());
//   Matrix matrix_equality_boundary =
//       Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());

//   std::unique_ptr<QpSolver> qp_solver(new ActiveSetQpSolver(
//       matrix_m1, matrix_m2, matrix_inequality_constrain,
//       matrix_inequality_boundary, matrix_equality_constrain,
//       matrix_equality_boundary));
//   auto result = qp_solver->Solve();
//   if (!result) {
//     // EERROR << "Linear MPC solver failed";
//     return false;
//   }
//   matrix_v = qp_solver->params();

//   for (unsigned int i = 0; i < horizon; ++i) {
//     (*control)[i] =
//         matrix_v.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1);
//   }
//   return true;
// }

}  // namespace math
}  // namespace common
}  // namespace EDrive
