
 float64 total_path_length # in meters
 float64 total_path_time # in seconds

#   # path data + speed data
 common/TrajectoryPoint[] trajectory_point

#  EStop estop

#   # path point without speed info
#  PathPoint path_point

#   # is_replan == true mean replan triggered
#  bool is_replan = false

#   # Specify trajectory gear
# #  apollo.canbus.Chassis.GearPosition gear = 10;

# #  apollo.planning.DecisionResult decision = 14;

#  LatencyStats latency_stats

#   # the routing used for current planning result
# #  apollo.common.Header routing_header = 16;
# #  apollo.planning_internal.Debug debug = 8;

# #   # replaced by path_point
# #  ADCPathPoint adc_path_point = 7 [deprecated=true];
# #   # replaced by trajectory_point
# #  ADCTrajectoryPoint adc_trajectory_point = 4 [deprecated=true];
# #  apollo.common.VehicleSignal signal = 11 [deprecated=true];
  
#  RightOfWayStatus right_of_way_status

#   # lane id along reference line
# #  apollo.hdmap.Id lane_id = 18;

# #   # set the engage advice for based on current planning result.
# #  apollo.common.EngageAdvice engage_advice = 19;

#   # the region where planning cares most
# #   message CriticalRegion {
# #    apollo.common.Polygon region = 1;
# #   }

# #   # critial region will be empty when planning is NOT sure which region is critical
# #   # critial regions may or may not overlap
# #  CriticalRegion critical_region = 20;