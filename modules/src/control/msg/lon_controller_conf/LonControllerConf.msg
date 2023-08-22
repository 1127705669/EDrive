 float64 ts
 float64 brake_minimum_action
 float64 throttle_minimum_action
 float64 speed_controller_input_limit
 float64 station_error_limit
 float64 preview_window
 float64 standstill_acceleration

 PidConf station_pid_conf
 PidConf low_speed_pid_conf
 PidConf high_speed_pid_conf
 float64 switch_speed
 PidConf reverse_station_pid_conf
 PidConf reverse_speed_pid_conf
 FilterConf pitch_angle_filter_conf
 
 bool enable_reverse_leadlag_compensation

 float64 switch_speed_window