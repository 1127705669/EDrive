  # coordinates
 float64 x
 float64 y
 float64 z

  # direction on the x-y plane
 float64 theta
  # curvature on the x-y planning
 float64 kappa
  # accumulated distance from beginning of the path
 float64 s

  # derivative of kappa w.r.t s.
 float64 dkappa
  # derivative of derivative of kappa w.r.t s.
 float64 ddkappa
  # The lane ID where the path point is on
 string lane_id