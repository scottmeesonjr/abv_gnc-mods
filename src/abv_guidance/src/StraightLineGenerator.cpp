
#include "abv_guidance/StraightLineGenerator.h"

StraightLineGenerator::StraightLineGenerator(const Waypoint& aGoal) : mGoal(aGoal)
{

}

StraightLineGenerator::~StraightLineGenerator()
{

}

bool StraightLineGenerator::init()
{
    mHasNext = true;
    return true;  
}

bool StraightLineGenerator::hasNext()
{
    return mHasNext; 
}

Waypoint StraightLineGenerator::getNext()
{
    if(mHasNext)
    {
        mHasNext = false;
        return mGoal;
    }
}

std::vector<Waypoint> StraightLineGenerator::getPath() const
{
    // No synthetic "start" point here - a pose captured once at construction
    // goes stale as soon as the robot moves. The GUI anchors the drawn line
    // to the robot's own live position instead (see TableTopView::drawPath).
    return {mGoal};
}

std::size_t StraightLineGenerator::getPathPreviewLength() const
{
    // getPath() always returns exactly {mGoal}.
    return 1;
}