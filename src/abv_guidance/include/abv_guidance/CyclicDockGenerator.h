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
//   2. arm       tell the mechanism to get ready for capture
//   3. approach  creep straight in at ApproachVelocity, no lateral correction;
//                ends on lock, at CaptureRange, or after ApproachTimeout
//   4. hold      stop thrusting and stay docked for DwellTime
//   5. unlock    release
//   6. hold      short pause so the latch opens before backing off
//
// After the last cycle it backs off to the standoff point and disarms.
//
// Test offsets come from the "dock" command's goal: goal x = radial offset (m,
// across the docking axis), goal yaw = angular offset (rad). goal y is unused.
// A zero goal gives a nominal (centered, aligned) run.
class CyclicDockGenerator : public IPathGenerator
{
public:
    CyclicDockGenerator(double aRadialOffset, double aAngularOffset, const Scene& aScene);
    ~CyclicDockGenerator() override;

    bool init() override;
    bool hasNext() override;
    Waypoint getNext() override;
    std::vector<Waypoint> getPath() const override;
    std::size_t getPathPreviewLength() const override;

private:
    double mRadialOffset;
    double mAngularOffset;
    const Scene& mScene;
    DockingConfig mConfig;

    std::size_t mIndex;
    std::vector<Waypoint> mPath;
};
#endif // CYCLICDOCKGENERATOR_H
