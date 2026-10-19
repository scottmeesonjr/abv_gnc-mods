#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include "abv_msgs/msg/abv_obstacle_array.hpp"

#include "abv_common/ThreadSafe.hpp"

#include "abv_guidance/AxisAlignedBoundingBox.hpp"
#include "abv_guidance/SceneTypes.hpp"

// Holds the 2D scene the ABV operates in: the fixed tabletop bounds (from
// SceneConfig) and the live set of obstacles reported on abv/scene/obstacles
// (e.g. from abv_gui's click-to-place stub). Intended to be queried by a
// future CollisionAvoidPathGenerator when evaluating candidate poses/paths.
class Scene
{
public:
    Scene();
    ~Scene();

    // the fixed tabletop bounds - the region isWithinBounds() checks against
    const AxisAlignedBoundingBox& getBounds() const { return mBounds; }

    // true iff aBox lies fully within the configured scene limits
    bool isWithinBounds(const AxisAlignedBoundingBox& aBox) const;

    // true iff aBox does not overlap any currently known obstacle
    bool isCollisionFree(const AxisAlignedBoundingBox& aBox) const;

    std::vector<Obstacle> getObstacles() const;

private:
    void obstaclesCallback(abv_msgs::msg::AbvObstacleArray::SharedPtr aMsg);

    AxisAlignedBoundingBox mBounds;
    ThreadSafe<std::vector<Obstacle>> mObstacles;
};
#endif // SCENE_H
