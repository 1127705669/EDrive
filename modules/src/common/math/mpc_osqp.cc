/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/mpc_osqp.h"

namespace EDrive {
namespace common {
namespace math {
MpcOsqp::MpcOsqp(const Eigen::MatrixXd &matrix_a,
                 const Eigen::MatrixXd &matrix_b,
                 const Eigen::MatrixXd &matrix_q,
                 const Eigen::MatrixXd &matrix_r,
                 const Eigen::MatrixXd &matrix_initial_x,
                 const Eigen::MatrixXd &matrix_u_lower,
                 const Eigen::MatrixXd &matrix_u_upper,
                 const Eigen::MatrixXd &matrix_x_lower,
                 const Eigen::MatrixXd &matrix_x_upper,
                 const Eigen::MatrixXd &matrix_x_ref, const int max_iter,
                 const int horizon, const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs) {
  state_dim_ = matrix_b.rows();
  control_dim_ = matrix_b.cols();
  EDEBUG << "state_dim" << state_dim_;
  EDEBUG << "control_dim_" << control_dim_;
  num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
}

void MpcOsqp::CalculateKernel(std::vector<c_float> *P_data,
                              std::vector<c_int> *P_indices,
                              std::vector<c_int> *P_indptr) {
  // col1:(row,val),...; col2:(row,val),....; ...
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_param_);
  size_t value_index = 0;
  // state and terminal state
  for (size_t i = 0; i <= horizon_; ++i) {
    for (size_t j = 0; j < state_dim_; ++j) {
      // (row, val)
      columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j,
                                               matrix_q_(j, j));
      ++value_index;
    }
  }
  // control
  const size_t state_total_dim = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < control_dim_; ++j) {
      // (row, val)
      columns[i * control_dim_ + j + state_total_dim].emplace_back(
          state_total_dim + i * control_dim_ + j, matrix_r_(j, j));
      ++value_index;
    }
  }
  CHECK_EQ(value_index, num_param_);

  int ind_p = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    // TODO(SHU) Check this
    P_indptr->emplace_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      P_data->emplace_back(row_data_pair.second);    // val
      P_indices->emplace_back(row_data_pair.first);  // row
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);
}

// reference is always zero
void MpcOsqp::CalculateGradient() {
  // populate the gradient vector
  gradient_ = Eigen::VectorXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_ + 1; i++) {
    gradient_.block(i * state_dim_, 0, state_dim_, 1) =
        -1.0 * matrix_q_ * matrix_x_ref_;
  }
  EDEBUG << "Gradient_mat";
  EDEBUG << gradient_;
}

// equality constraints x(k+1) = A*x(k)
void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                          std::vector<c_int> *A_indices,
                                          std::vector<c_int> *A_indptr) {
  static constexpr double kEpsilon = 1e-6;
  // block matrix
  Eigen::MatrixXd matrix_constraint = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
          control_dim_ * horizon_,
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);
  Eigen::MatrixXd state_identity_mat = Eigen::MatrixXd::Identity(
      state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1));
  EDEBUG << "state_identity_mat" << state_identity_mat;

  matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1),
                          state_dim_ * (horizon_ + 1)) =
      -1 * state_identity_mat;
  EDEBUG << "matrix_constraint";
  EDEBUG << matrix_constraint;

  Eigen::MatrixXd control_identity_mat =
      Eigen::MatrixXd::Identity(control_dim_, control_dim_);

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, state_dim_,
                            state_dim_) = matrix_a_;
  }
  EDEBUG << "matrix_constraint with A";
  EDEBUG << matrix_constraint;

  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_,
                            i * control_dim_ + (horizon_ + 1) * state_dim_,
                            state_dim_, control_dim_) = matrix_b_;
  }
  EDEBUG << "matrix_constraint with B";
  EDEBUG << matrix_constraint;

  Eigen::MatrixXd all_identity_mat =
      Eigen::MatrixXd::Identity(num_param_, num_param_);

  matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, num_param_,
                          num_param_) = all_identity_mat;
  EDEBUG << "matrix_constraint with I";
  EDEBUG << matrix_constraint;

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_param_ + 1);
  int value_index = 0;
  // state and terminal state
  for (size_t i = 0; i < num_param_; ++i) {  // col
    for (size_t j = 0; j < num_param_ + state_dim_ * (horizon_ + 1);
         ++j)  // row
      if (std::fabs(matrix_constraint(j, i)) > kEpsilon) {
        // (row, val)
        columns[i].emplace_back(j, matrix_constraint(j, i));
        ++value_index;
      }
  }
  EDEBUG << "value_index";
  EDEBUG << value_index;
  int ind_A = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    A_indptr->emplace_back(ind_A);
    for (const auto &row_data_pair : columns[i]) {
      A_data->emplace_back(row_data_pair.second);    // value
      A_indices->emplace_back(row_data_pair.first);  // row
      ++ind_A;
    }
  }
  A_indptr->emplace_back(ind_A);
}

void MpcOsqp::CalculateConstraintVectors() {
  // evaluate the lower and the upper inequality vectors
  Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_; i++) {
    lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_lower_;
    upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_upper_;
  }
  EDEBUG << " matrix_u_lower_";
  for (size_t i = 0; i < horizon_ + 1; i++) {
    lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
    upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
  }
  EDEBUG << " matrix_x_lower_";

  // evaluate the lower and the upper equality vectors
  Eigen::VectorXd lowerEquality =
      Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd upperEquality;
  lowerEquality.block(0, 0, state_dim_, 1) = -1 * matrix_initial_x_;
  upperEquality = lowerEquality;
  lowerEquality = lowerEquality;
  EDEBUG << " matrix_initial_x_";

  // merge inequality and equality vectors
  lowerBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  lowerBound_ << lowerEquality, lowerInequality;
  EDEBUG << " lowerBound_ ";
  upperBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  upperBound_ << upperEquality, upperInequality;
  EDEBUG << " upperBound_";
}

OSQPSettings *MpcOsqp::Settings() {
  // default setting
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  if (settings == nullptr) {
    return nullptr;
  } else {
    osqp_set_default_settings(settings);
    settings->polish = true;
    settings->scaled_termination = true;
    settings->verbose = false;
    settings->max_iter = max_iteration_;
    settings->eps_abs = eps_abs_;
    return settings;
  }
}

OSQPData *MpcOsqp::Data() {
  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  size_t num_affine_constraint =
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  if (data == nullptr) {
    return nullptr;
  } else {
    data->n = kernel_dim;
    data->m = num_affine_constraint;
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    EDEBUG << "before CalculateKernel";
    CalculateKernel(&P_data, &P_indices, &P_indptr);
    EDEBUG << "CalculateKernel done";
    data->P =
        csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                   CopyData(P_indices), CopyData(P_indptr));
    EDEBUG << "Get P matrix";
    data->q = gradient_.data();
    EDEBUG << "before CalculateEqualityConstraint";
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);
    EDEBUG << "CalculateEqualityConstraint done";
    data->A =
        csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
                       control_dim_ * horizon_,
                   kernel_dim, A_data.size(), CopyData(A_data),
                   CopyData(A_indices), CopyData(A_indptr));
    EDEBUG << "Get A matrix";
    data->l = lowerBound_.data();
    data->u = upperBound_.data();
    return data;
  }
}

void MpcOsqp::FreeData(OSQPData *data) {
  c_free(data->A);
  c_free(data->P);
  c_free(data);
}

bool MpcOsqp::Solve(std::vector<double> *control_cmd) {
  EDEBUG << "Before Calc Gradient";
  CalculateGradient();
  EDEBUG << "After Calc Gradient";
  CalculateConstraintVectors();
  EDEBUG << "MPC2Matrix";

  OSQPData *data = Data();
  EDEBUG << "OSQP data done";
  EDEBUG << "OSQP data n" << data->n;
  EDEBUG << "OSQP data m" << data->m;
  for (int i = 0; i < data->n; ++i) {
    EDEBUG << "OSQP data q" << i << ":" << (data->q)[i];
  }
  EDEBUG << "OSQP data l" << data->l;
  for (int i = 0; i < data->m; ++i) {
    EDEBUG << "OSQP data l" << i << ":" << (data->l)[i];
  }
  EDEBUG << "OSQP data u" << data->u;
  for (int i = 0; i < data->m; ++i) {
    EDEBUG << "OSQP data u" << i << ":" << (data->u)[i];
  }

  OSQPSettings *settings = Settings();
  EDEBUG << "OSQP setting done";
  OSQPWorkspace *osqp_workspace = nullptr;
  // osqp_setup(&osqp_workspace, data, settings);
  osqp_setup(&osqp_workspace, data, settings);
  // osqp_workspace = osqp_setup(data, settings);
  EDEBUG << "OSQP workspace ready";
  osqp_solve(osqp_workspace);

  auto status = osqp_workspace->info->status_val;
  EDEBUG << "status:" << status;
  // check status
  if (status < 0 || (status != 1 && status != 2)) {
    EERROR << "failed optimization status:\t" << osqp_workspace->info->status;
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_workspace->solution == nullptr) {
    EERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  }

  size_t first_control = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < control_dim_; ++i) {
    control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
    EDEBUG << "control_cmd:" << i << ":" << control_cmd->at(i);
  }

  // Cleanup
  osqp_cleanup(osqp_workspace);
  FreeData(data);
  c_free(settings);
  return true;
}

}  // namespace math
}  // namespace common
}  // namespace EDrive
