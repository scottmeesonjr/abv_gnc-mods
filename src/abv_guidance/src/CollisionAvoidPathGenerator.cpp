
#include "abv_guidance/CollisionAvoidPathGenerator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "plog/Log.h"

#include "abv_common/ConfigurationManager.h"
#include "abv_common/RateController.hpp"

namespace
{
    using Cell = std::pair<int, int>;

    struct CellHash
    {
        std::size_t operator()(const Cell& aCell) const
        {
            std::size_t h1 = std::hash<int>{}(aCell.first);
            std::size_t h2 = std::hash<int>{}(aCell.second);
            return h1 ^ (h2 << 1);
        }
    };

    struct QueueEntry
    {
        double mFScore;
        Cell mCell;
    };

    struct QueueEntryCompare
    {
        bool operator()(const QueueEntry& aLhs, const QueueEntry& aRhs) const
        {
            return aLhs.mFScore > aRhs.mFScore;
        }
    };

    constexpr int kNeighborOffsets[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
}

CollisionAvoidPathGenerator::CollisionAvoidPathGenerator(const Waypoint& aGoal, RosNavigationListener& aNavSource, const Scene& aScene)
    : mGoal(aGoal), mNavSource(aNavSource), mScene(aScene), mTrajHeading(0.0)
{
    mGridResolution = ConfigurationManager::getInstance()->getCollisionAvoidConfig().mGridResolution;

    const TableViewConfig& tableConfig = ConfigurationManager::getInstance()->getTableViewConfig();

    // Heading-agnostic grid search (see header comment) - use a conservative
    // circumscribing square (max of the two half-extents) rather than assume
    // an orientation, so the collision check never under-estimates the
    // footprint regardless of the robot's actual heading.
    mRobotHalfExtent = std::max(tableConfig.mRobotWidth, tableConfig.mRobotLength) / 2.0;
}

CollisionAvoidPathGenerator::~CollisionAvoidPathGenerator()
{
    mDone = true;
    if(mThread.joinable())
    {
        mThread.join();
    }
}

bool CollisionAvoidPathGenerator::init()
{
    Eigen::Vector3d start = mNavSource.getCurrentPose();
    
    // relative position vector from start to goal 
    Eigen::Vector2d p_G_S = Eigen::Vector2d(mGoal.mPose.x() - start.x(), mGoal.mPose.y() - start.y()); 
    double relativeHeading = atan2(p_G_S.y(), p_G_S.x()); 
    mTrajHeading = abs(relativeHeading) < M_PI/2 ? 0.0 : M_PI;
    
    std::vector<Waypoint> path = solveAStar(start, mGoal.mPose);

    if(path.empty())
    {
        LOGW << "CollisionAvoidPathGenerator: failed to find an initial collision-free path";
        return false;
    }

    mLatestPath.set(path);
    mThread = std::thread(&CollisionAvoidPathGenerator::planningLoop, this);

    return true;
}

bool CollisionAvoidPathGenerator::hasNext()
{
    Eigen::Vector3d current = mNavSource.getCurrentPose();
    double distToGoal = (mGoal.mPose.head<2>() - current.head<2>()).norm();
    return distToGoal > mGridResolution;
}

Waypoint CollisionAvoidPathGenerator::getNext()
{
    std::vector<Waypoint> path = mLatestPath.get();
    if(path.size() < 2)
    {
        return mGoal;
    }

    return path[1];
}

std::vector<Waypoint> CollisionAvoidPathGenerator::getPath() const
{
    // Drop the leading node - it's "current position as of the last solve",
    // which goes stale by the time this is read (same lesson already applied
    // to StraightLineGenerator). abv_gui anchors the drawn line to the
    // robot's own live pose itself, so nothing is lost.
    std::vector<Waypoint> path = mLatestPath.get();
    if(path.size() < 2)
    {
        return {};
    }

    return std::vector<Waypoint>(path.begin() + 1, path.end());
}

std::size_t CollisionAvoidPathGenerator::getPathPreviewLength() const
{
    // "show everything" - StateMachine clamps this to the actual size.
    return std::numeric_limits<std::size_t>::max();
}

void CollisionAvoidPathGenerator::planningLoop()
{
    double replanRate = ConfigurationManager::getInstance()->getCollisionAvoidConfig().mReplanRate;
    RateController rate(static_cast<int>(replanRate));

    while(!mDone)
    {
        rate.start();

        Eigen::Vector3d current = mNavSource.getCurrentPose();
        std::vector<Waypoint> path = solveAStar(current, mGoal.mPose);

        if(!path.empty())
        {
            mLatestPath.set(path);
        }
        else
        {
            LOGW << "CollisionAvoidPathGenerator: no collision-free path found this cycle, keeping previous plan";
        }

        rate.block();
    }
}

std::pair<int, int> CollisionAvoidPathGenerator::worldToCell(double aX, double aY) const
{
    const AxisAlignedBoundingBox& bounds = mScene.getBounds();
    int col = static_cast<int>(std::floor((aX - bounds.mXMin) / mGridResolution));
    int row = static_cast<int>(std::floor((aY - bounds.mYMin) / mGridResolution));
    return {col, row};
}

Eigen::Vector2d CollisionAvoidPathGenerator::cellToWorld(int aCol, int aRow) const
{
    const AxisAlignedBoundingBox& bounds = mScene.getBounds();
    double x = bounds.mXMin + (aCol + 0.5) * mGridResolution;
    double y = bounds.mYMin + (aRow + 0.5) * mGridResolution;
    return Eigen::Vector2d(x, y);
}

bool CollisionAvoidPathGenerator::isCellFree(int aCol, int aRow) const
{
    Eigen::Vector2d center = cellToWorld(aCol, aRow);
    AxisAlignedBoundingBox robotBox{
        center.x() - mRobotHalfExtent, center.x() + mRobotHalfExtent,
        center.y() - mRobotHalfExtent, center.y() + mRobotHalfExtent
    };

    return mScene.isWithinBounds(robotBox) && mScene.isCollisionFree(robotBox);
}

std::vector<Waypoint> CollisionAvoidPathGenerator::solveAStar(const Eigen::Vector3d& aStart, const Eigen::Vector3d& aGoalPose) const
{
    Cell startCell = worldToCell(aStart.x(), aStart.y());
    Cell goalCell = worldToCell(aGoalPose.x(), aGoalPose.y());

    if(!mScene.getBounds().contains(AxisAlignedBoundingBox::fromCircle(aGoalPose.x(), aGoalPose.y(), 0.0)))
    {
        LOGW << "CollisionAvoidPathGenerator: goal lies outside the configured scene bounds";
        return {};
    }

    // The start cell is exempt from its own collision check below (the robot
    // can't retroactively un-occupy where it already is) - only the goal and
    // cells being expanded into need to be free.
    if(!isCellFree(goalCell.first, goalCell.second))
    {
        LOGW << "CollisionAvoidPathGenerator: goal cell is not collision-free";
        return {};
    }

    auto cellDistance = [this](const Cell& aLhs, const Cell& aRhs) {
        double dx = (aLhs.first - aRhs.first) * mGridResolution;
        double dy = (aLhs.second - aRhs.second) * mGridResolution;
        return std::sqrt(dx * dx + dy * dy);
    };

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> openSet;
    std::unordered_map<Cell, double, CellHash> gScore;
    std::unordered_map<Cell, Cell, CellHash> cameFrom;
    std::unordered_set<Cell, CellHash> closedSet;

    gScore[startCell] = 0.0;
    openSet.push({cellDistance(startCell, goalCell), startCell});

    bool found = false;

    while(!openSet.empty())
    {
        Cell current = openSet.top().mCell;
        openSet.pop();

        if(closedSet.count(current))
        {
            continue;
        }

        if(current == goalCell)
        {
            found = true;
            break;
        }

        closedSet.insert(current);

        for(const auto& offset : kNeighborOffsets)
        {
            Cell neighbor{current.first + offset[0], current.second + offset[1]};

            if(closedSet.count(neighbor))
            {
                continue;
            }

            if(neighbor != startCell && !isCellFree(neighbor.first, neighbor.second))
            {
                continue;
            }

            double tentativeG = gScore[current] + cellDistance(current, neighbor);

            auto it = gScore.find(neighbor);
            if(it == gScore.end() || tentativeG < it->second)
            {
                gScore[neighbor] = tentativeG;
                cameFrom[neighbor] = current;
                openSet.push({tentativeG + cellDistance(neighbor, goalCell), neighbor});
            }
        }
    }

    if(!found)
    {
        return {};
    }

    std::vector<Cell> cellPath;
    Cell walk = goalCell;
    while(!(walk == startCell))
    {
        cellPath.push_back(walk);
        walk = cameFrom.at(walk);
    }
    cellPath.push_back(startCell);
    std::reverse(cellPath.begin(), cellPath.end());

    std::vector<Waypoint> path;
    path.reserve(cellPath.size());

    for(std::size_t i = 0; i < cellPath.size(); ++i)
    {
        Eigen::Vector2d worldPt = cellToWorld(cellPath[i].first, cellPath[i].second);
        path.emplace_back(worldPt.x(), worldPt.y(), mTrajHeading, mGoal.mType);
    }

    // Exact commanded goal pose for the final waypoint, not the goal cell's
    // approximate center - so the vehicle ends up exactly where commanded.
    path.back() = Waypoint(aGoalPose.x(), aGoalPose.y(), aGoalPose.z(), mGoal.mType);

    return path;
}
