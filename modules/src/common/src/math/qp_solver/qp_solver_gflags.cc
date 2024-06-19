/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/src/math/qp_solver/qp_solver_gflags.h"

// math : active set solver
DEFINE_double(default_active_set_eps_num, -1e-7,
              "qpOases wrapper error control numerator");
DEFINE_double(default_active_set_eps_den, 1e-7,
              "qpOases wrapper error control denominator");
DEFINE_double(default_active_set_eps_iter_ref, 1e-7,
              "qpOases wrapper early termination tolerance for iterative refinement");  // NOLINT
DEFINE_bool(default_enable_active_set_debug_info, false,
            "Enable print information");
DEFINE_int32(default_qp_iteration_num, 1000, "Default qp oases iteration time");
