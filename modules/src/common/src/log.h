/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 */

#pragma once

#include <ros/ros.h>
#include <cstdlib> // For std::abort()

#include "glog/logging.h"
#include "glog/raw_logging.h"

// Define the MODULE_NAME macro if not already defined
#ifndef MODULE_NAME
#define MODULE_NAME "EDrive"
#endif

// Define brackets for consistent formatting
#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

// Define the EDEBUG_MODULE macro to use Google VLOG for logging
#define EDEBUG_MODULE(module) \
  VLOG(4) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG] "
#define EDEBUG EDEBUG_MODULE(MODULE_NAME)
#define EINFO ELOG_MODULE(MODULE_NAME, INFO)
#define EWARN ELOG_MODULE(MODULE_NAME, WARN)
#define EERROR ELOG_MODULE(MODULE_NAME, ERROR)
#define EFATAL ELOG_MODULE(MODULE_NAME, FATAL)

#ifndef ELOG_MODULE_STREAM
#define ELOG_MODULE_STREAM(log_severity) ELOG_MODULE_STREAM_##log_severity
#endif

#ifndef ELOG_MODULE
#define ELOG_MODULE(module, log_severity) \
  ELOG_MODULE_STREAM(log_severity)(module)
#endif

#define ELOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ELOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ELOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ELOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define EINFO_IF(cond) ELOG_IF(INFO, cond, MODULE_NAME)
#define EWARN_IF(cond) ELOG_IF(WARN, cond, MODULE_NAME)
#define EERROR_IF(cond) ELOG_IF(ERROR, cond, MODULE_NAME)
#define EFATAL_IF(cond) ELOG_IF(FATAL, cond, MODULE_NAME)
#define ELOG_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
          : google::LogMessageVoidify() & ELOG_MODULE(module, severity)

#define ECHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define EINFO_EVERY(freq) \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define EWARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define EERROR_EVERY(freq) \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET


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
