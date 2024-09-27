/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "gflags/gflags.h"
// controller pipeline file path
DECLARE_string(pipeline_file);
// data file
DECLARE_string(control_conf_file);
// mpc controller conf file
DECLARE_string(mpc_controller_conf_file);
// lateral controller conf file
DECLARE_string(lateral_controller_conf_file);
// longitudinal controller conf file
DECLARE_string(longitudinal_controller_conf_file);
// calibration table
DECLARE_string(calibration_table_file);

DECLARE_double(control_test_duration);

DECLARE_bool(enable_csv_debug);

DECLARE_string(preprocessor_submodule_name);
DECLARE_string(mpc_controller_submodule_name);
DECLARE_string(lat_lon_controller_submodule_name);
DECLARE_string(postprocessor_submodule_name);

DECLARE_bool(is_control_test_mode);
DECLARE_bool(use_preview_speed_for_table);

DECLARE_double(steer_angle_rate);
DECLARE_bool(enable_gain_scheduler);
DECLARE_bool(set_steer_limit);

DECLARE_int32(chassis_pending_queue_size);
DECLARE_int32(planning_pending_queue_size);
DECLARE_int32(localization_pending_queue_size);
DECLARE_int32(pad_msg_pending_queue_size);

DECLARE_bool(reverse_heading_control);

DECLARE_bool(query_forward_time_point_only);

DECLARE_bool(enable_gear_drive_negative_speed_protection);

DECLARE_bool(use_control_submodules);

DECLARE_bool(enable_input_timestamp_check);

DECLARE_int32(max_localization_miss_num);

DECLARE_int32(max_chassis_miss_num);

DECLARE_int32(max_planning_miss_num);

DECLARE_int32(planning_status_msg_pending_queue_size);

DECLARE_double(max_acceleration_when_stopped);

DECLARE_double(max_path_remain_when_stopped);

DECLARE_bool(enable_persistent_estop);

DECLARE_double(control_period);

DECLARE_double(soft_estop_brake);

DECLARE_double(trajectory_period);

DECLARE_double(chassis_period);

DECLARE_double(localization_period);

DECLARE_double(minimum_speed_resolution);

DECLARE_double(minimum_speed_protection);

// 0: stop, at first should stop, then receive pad msg to start
DECLARE_int32(action);

DECLARE_bool(use_vehicle_epb);

DECLARE_double(pitch_offset_deg);

DECLARE_bool(enable_feedback_augment_on_high_speed);

DECLARE_bool(is_control_ut_test_mode);

DECLARE_bool(publish_control_debug_info);

DECLARE_bool(enable_maximum_steer_rate_limit);

DECLARE_bool(enable_navigation_mode_error_filter);