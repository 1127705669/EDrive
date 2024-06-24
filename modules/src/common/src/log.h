// /******************************************************************************
//  * Copyright 2024 The EDrive Authors. All Rights Reserved.
//  *****************************************************************************/

// /**
//  * @file
//  */

// #ifndef MODULES_COMMON_LOG_H_
// #define MODULES_COMMON_LOG_H_

// #include "glog/logging.h"
// #include "glog/raw_logging.h"

// #define EDEBUG VLOG(4) << "[DEBUG] "
// #define EINFO LOG(INFO)
// #define EWARN LOG(WARNING)
// #define EERROR LOG(ERROR)
// #define EFATAL LOG(FATAL)

// // LOG_IF
// #define EINFO_IF(cond) LOG_IF(INFO, cond)
// #define EERROR_IF(cond) LOG_IF(ERROR, cond)
// #define ECHECK(cond) CHECK(cond)

// // LOG_EVERY_N
// #define EINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
// #define EWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
// #define EERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

// #define RETURN_IF_NULL(ptr)               \
//     if (ptr == nullptr) {                 \
//         EWARN << #ptr << " is nullptr.";  \
//         return;                           \
//     }

// #define RETURN_VAL_IF_NULL(ptr, val)      \
//     if (ptr == nullptr) {                 \
//         EWARN << #ptr << " is nullptr.";  \
//         return val;                       \
//     }

// #define RETURN_IF(condition)                   \
//     if (condition) {                           \
//         EWARN << #condition << " is not met."; \
//         return;                                \
//     }

// #define RETURN_VAL_IF(condition, val)          \
//     if (condition) {                           \
//         EWARN << #condition << " is not met."; \
//         return val;                            \
//     }

// #endif  // MODULES_COMMON_LOG_H_
