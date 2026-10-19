# Velocity alert (SYS.01), published by abv_navigation on /abv/nav/alert at the nav rate.

builtin_interfaces/Time stamp
float64 speed              # m/s, sqrt(vx^2 + vy^2); this is what the alert compares
float64 closing_velocity   # m/s, velocity along the docking axis, + = toward the target
float64 range              # m, gap between the ABV's active face and the passive face
bool alert                 # set above Navigation.VelocityAlertThreshold, cleared below VelocityAlertClear
