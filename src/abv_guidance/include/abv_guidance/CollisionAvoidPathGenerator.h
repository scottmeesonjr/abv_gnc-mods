#ifndef COLLISIONAVOIDPATHGENERATOR_H
#define COLLISIONAVOIDPATHGENERATOR_H

#include <atomic>
#include <thread>
#include <utility>
#include <eigen3/Eigen/Dense>

#include "abv_common/RosNavigationListener.h"
#include "abv_common/ThreadSafe.hpp"

#include "abv_guidance/IPathGenerator.hpp"
#include "abv_guidance/Scene.h"

// A*-based collision-avoiding path generator: a background thread
// continuously re-solves a grid search from the robot's current live pose to
// a fixed goal, using Scene's AABB collision checks (obstacles + table
// bounds), so the plan stays responsive to obstacles arriving/moving on
// abv/scene/obstacles. The grid search is heading-agnostic - the robot
// footprint is checked as an axis-aligned box regardless of orientation
// (conservative circumscribing square, see the constructor), consistent with
// the rest of this collision system's "simple AABB" scope.
class CollisionAvoidPathGenerator : public IPathGenerator
{
public:
    CollisionAvoidPathGenerator(const Waypoint& aGoal, RosNavigationListener& aNavSource, const Scene& aScene);
    ~CollisionAvoidPathGenerator() override;

    bool init() override;
    bool hasNext() override;
    Waypoint getNext() override;
    std::vector<Waypoint> getPath() const override;
    std::size_t getPathPreviewLength() const override;

private:
    void planningLoop();

    // Full start-to-goal collision-free path, or empty if none found.
    std::vector<Waypoint> solveAStar(const Eigen::Vector3d& aStart, const Eigen::Vector3d& aGoalPose) const;

    bool isCellFree(int aCol, int aRow) const;
    Eigen::Vector2d cellToWorld(int aCol, int aRow) const;
    std::pair<int, int> worldToCell(double aX, double aY) const;

    Waypoint mGoal;
    RosNavigationListener& mNavSource;
    const Scene& mScene;
    double mTrajHeading; 

    double mGridResolution;
    double mRobotHalfExtent;

    std::atomic<bool> mDone{false};
    std::thread mThread;

    // Latest full A* solve, start-to-goal (index 0 is "current position as of
    // that solve" - stale by the time it's read, see getPath()/getNext()).
    ThreadSafe<std::vector<Waypoint>> mLatestPath;
};
#endif // COLLISIONAVOIDPATHGENERATOR_H
