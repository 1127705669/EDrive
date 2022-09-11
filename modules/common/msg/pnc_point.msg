message TrajectoryPoint {
  // path point
  optional PathPoint path_point = 1;

  // linear velocity
  optional double v = 2;  // in [m/s]
  // linear acceleration
  optional double a = 3;
  // relative time from beginning of the trajectory
  optional double relative_time = 4;
}