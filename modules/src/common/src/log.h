/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 */

#pragma once

#include <ros/ros.h>
#include <cstdlib> // For std::abort()

// Define logging macros to encapsulate ROS logging macros
#define EINFO(...) ROS_INFO(__VA_ARGS__)
#define EWARN(...) ROS_WARN(__VA_ARGS__)
#define EERROR(...) ROS_ERROR(__VA_ARGS__)
#define EFATAL(...) ROS_FATAL(__VA_ARGS__)

// Define conditional logging macros
#define EINFO_IF(cond, ...) if (cond) { ROS_INFO(__VA_ARGS__); }
#define EERROR_IF(cond, ...) if (cond) { ROS_ERROR(__VA_ARGS__); }

// Define check macros
#define ECHECK(cond) \
  if (!(cond)) { \
    ROS_FATAL("Check failed: %s", #cond); \
    std::abort(); \
  }

#define CHECK_OP(name, op, val1, val2) \
  if (!((val1) op (val2))) { \
    ROS_FATAL("Check failed: %s %s %s (%zu vs. %zu)", #val1, #op, #val2, (size_t)(val1), (size_t)(val2)); \
    std::abort(); \
  }

#define ECHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define ECHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define ECHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define ECHECK_LT(val1, val2) CHECK_OP(_LT, < , val1, val2)
#define ECHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define ECHECK_GT(val1, val2) CHECK_OP(_GT, > , val1, val2)

// Define frequency logging macros
#define EINFO_EVERY(freq, ...) do { static int count = 0; if (++count % (freq) == 0) { ROS_INFO(__VA_ARGS__); } } while (0)
#define EWARN_EVERY(freq, ...) do { static int count = 0; if (++count % (freq) == 0) { ROS_WARN(__VA_ARGS__); } } while (0)
#define EERROR_EVERY(freq, ...) do { static int count = 0; if (++count % (freq) == 0) { ROS_ERROR(__VA_ARGS__); } } while (0)

// Define return condition macros
#define RETURN_IF_NULL(ptr)               \
  if (ptr == nullptr) {                 \
    EWARN("%s is nullptr.", #ptr);    \
    return;                           \
  }

#define RETURN_VAL_IF_NULL(ptr, val)      \
  if (ptr == nullptr) {                 \
    EWARN("%s is nullptr.", #ptr);    \
    return val;                       \
  }

#define RETURN_IF(condition)                   \
  if (condition) {                           \
    EWARN("%s is not met.", #condition);   \
    return;                                \
  }

#define RETURN_VAL_IF(condition, val)          \
  if (condition) {                           \
    EWARN("%s is not met.", #condition);   \
    return val;                            \
  }
