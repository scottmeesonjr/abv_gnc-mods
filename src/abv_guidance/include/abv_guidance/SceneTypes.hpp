#ifndef SCENETYPES_HPP
#define SCENETYPES_HPP

// Internal, ROS-message-free obstacle representation - Scene converts
// AbvObstacle msgs to/from this on the wire, same role InternalTypes.hpp's
// Waypoint/Command play for the rest of abv_guidance.
struct Obstacle
{
    double mX;
    double mY;
    double mRadius;
};

#endif // SCENETYPES_HPP
