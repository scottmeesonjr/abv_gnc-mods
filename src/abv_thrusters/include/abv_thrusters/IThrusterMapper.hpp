#ifndef ITHRUSTERMAPPER_HPP
#define ITHRUSTERMAPPER_HPP

#include <eigen3/Eigen/Dense>
#include <string>

struct ThrusterAllocation
{
    std::string mThrusterCommand;         // 8-char '0'/'1' string
    Eigen::Vector3d mAppliedThrustVector; // resulting (fx, fy, tz)
};

class IThrusterMapper
{
public:
    virtual ~IThrusterMapper() = default;
    virtual ThrusterAllocation map(const Eigen::Vector3i& aThrustDir) = 0;
};
#endif //ITHRUSTERMAPPER_HPP
