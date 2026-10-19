
#ifndef GOAL_TYPE_HPP
#define GOAL_TYPE_HPP

// mirrors abv_goal_manager/GoalType.hpp and AbvControlContext.msg's goal_type
// field - kept as abv_control's own local copy rather than a cross-package
// header dependency between otherwise-independent nodes.
enum class GoalType
{
    THRUSTER,
    POSE,
    VELOCITY,
    NUM_TYPES
};

#endif
