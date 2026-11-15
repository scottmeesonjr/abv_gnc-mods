#ifndef CYCLICDOCKGENERATOR_H
#define CYCLICDOCKGENERATOR_H

#include <vector>

#include "abv_common/Configurations.h"
#include "abv_guidance/IPathGenerator.hpp"
#include "abv_guidance/Scene.h"

// Repeats the full docking cycle Docking.Cycles times, using the geometry
// and timing in config.yaml's Docking block. Each cycle:
//
//   1. pose      drive to the standoff point (tight StartTolerance)
//   2. hold      3 s at zero velocity so the approach starts from rest
//   3. arm       tell the mechanism to get ready for capture
//   4. approach  creep straight in at ApproachVelocity, no lateral correction;
//                ends on lock, at CaptureRange, or after ApproachTimeout
//   5. hold      hold zero velocity and stay docked for DwellTime
//   6. unlock    release
//   7. hold      short pause so the latch opens before backing off
//
// After the last cycle it backs off to the standoff point and disarms.
//
// Test settings come from the "dock" command's goal:
//   goal x   = radial offset (m, across the docking axis)        SYS.07 / TEST.04A
//   goal yaw = angular offset (rad)                               SYS.08 / TEST.04B
//   goal y   = approach speed (m/s); 0 = Docking.ApproachVelocity  SYS.10 / TEST.04C
//   goal velocity x = number of cycles; 0 = Docking.Cycles           SYS.06
// A zero goal gives a nominal (centered, aligned, config-speed) run.
class CyclicDockGenerator : public IPathGenerator
{
public:
    CyclicDockGenerator(double aRadialOffset, double aAngularOffset, double aApproachSpeed,
                        int aCycles, const Scene& aScene);
    int cycles() const { return mCycles; }
    ~CyclicDockGenerator() override;

    bool init() override;
    bool hasNext() override;
    Waypoint getNext() override;
    std::vector<Waypoint> getPath() const override;
    std::size_t getPathPreviewLength() const override;

private:
    double mRadialOffset;
    double mAngularOffset;
    double mApproachSpeed;
    int mCycles;
    const Scene& mScene;
    DockingConfig mConfig;

    std::size_t mIndex;
    std::vector<Waypoint> mPath;
};
#endif // CYCLICDOCKGENERATOR_H
