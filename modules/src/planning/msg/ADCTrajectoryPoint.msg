  float64 x  # in meters.accumulated_s
  float64 y  # in meters.
  float64 z  # height in meters.

  float64 speed  # speed, in meters / second
  float64 acceleration_s  # acceleration in s direction
  float64 curvature  # curvature (k = 1/r), unit: (1/meters)
  # change of curvature in unit s (dk/ds)
  float64 curvature_change_rate
  # in seconds (relative_time = time_of_this_state - timestamp_in_header)
  float64 relative_time
  float64 theta  # relative to absolute coordinate system
  # calculated from the first point in this trajectory
  float64 accumulated_s