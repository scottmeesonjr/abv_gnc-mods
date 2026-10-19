# Published by abv_goal_manager once per state-machine tick while in
# POSE_CONTROL/VELOCITY_CONTROL, so any control-law node (abv_control,
# abv_rl-cpp) can compute a control input without abv_goal_manager needing
# to know which/how many control-law nodes are listening.

AbvVec3 pose
AbvVec3 velocity
AbvVec3 goal
AbvVec3 error

# mirrors GoalType.hpp
uint8 POSE=0
uint8 VELOCITY=1

uint8 goal_type
