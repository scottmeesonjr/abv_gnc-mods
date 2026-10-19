
#include "abv_guidance/Scene.h"

#include "abv_common/ConfigurationManager.h"
#include "abv_common/RosTopicManager.h"

Scene::Scene()
{
    SceneConfig& config = ConfigurationManager::getInstance()->getSceneConfig();
    mBounds = AxisAlignedBoundingBox{config.mXMin, config.mXMax, config.mYMin, config.mYMax};

    RosTopicManager::getInstance()->createSubscriber<abv_msgs::msg::AbvObstacleArray>(
        "abv/scene/obstacles", std::bind(&Scene::obstaclesCallback, this, std::placeholders::_1));
}

Scene::~Scene()
{

}

bool Scene::isWithinBounds(const AxisAlignedBoundingBox& aBox) const
{
    return mBounds.contains(aBox);
}

bool Scene::isCollisionFree(const AxisAlignedBoundingBox& aBox) const
{
    std::vector<Obstacle> obstacles = mObstacles.get();
    for(const Obstacle& obstacle : obstacles)
    {
        AxisAlignedBoundingBox obstacleBox = AxisAlignedBoundingBox::fromCircle(obstacle.mX, obstacle.mY, obstacle.mRadius);
        if(aBox.overlaps(obstacleBox))
        {
            return false;
        }
    }

    return true;
}

std::vector<Obstacle> Scene::getObstacles() const
{
    return mObstacles.get();
}

void Scene::obstaclesCallback(abv_msgs::msg::AbvObstacleArray::SharedPtr aMsg)
{
    std::vector<Obstacle> obstacles;
    obstacles.reserve(aMsg->obstacles.size());

    for(const auto& obstacle : aMsg->obstacles)
    {
        obstacles.push_back(Obstacle{obstacle.x, obstacle.y, obstacle.radius});
    }

    mObstacles.set(obstacles);
}
