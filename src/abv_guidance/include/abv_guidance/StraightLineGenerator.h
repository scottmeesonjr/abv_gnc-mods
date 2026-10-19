#ifndef STRAIGHTLINEGENERATOR_H
#define STRAIGHTLINEGENERATOR_H
 
#include "abv_guidance/IPathGenerator.hpp" 
#include <eigen3/Eigen/Dense>

class StraightLineGenerator : public IPathGenerator
{
public:
    StraightLineGenerator(const Waypoint& aGoal);
    ~StraightLineGenerator() override;

    bool init() override;
    bool hasNext() override;
    Waypoint getNext() override;
    std::vector<Waypoint> getPath() const override;
    std::size_t getPathPreviewLength() const override;

private:

    Waypoint mGoal;

    bool mHasNext;

};
#endif //STRAIGHTLINEGENERATOR_H