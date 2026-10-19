#ifndef IPATHGENERATOR_HPP
#define IPATHGENERATOR_HPP

#include <vector>

#include "abv_guidance/InternalTypes.hpp"
 
class IPathGenerator 
{ 
public:

    virtual ~IPathGenerator() = default; 

    virtual bool init() = 0;
    virtual bool hasNext() = 0;
    virtual Waypoint getNext() = 0;

    // The remaining path from the current waypoint/state to the goal (or as
    // far ahead as this generator plans) - for visualization only, not
    // consumed like getNext(). Exactly what "remaining" means is up to each
    // implementation.
    virtual std::vector<Waypoint> getPath() const = 0;

    // How many waypoints from the front of getPath()'s result are actually
    // meaningful to publish/visualize right now. Generators that already
    // know their full remaining route (straight line, from-file) return
    // everything; an incremental/receding-horizon planner that only has N
    // steps currently planned returns just that N.
    virtual std::size_t getPathPreviewLength() const = 0;

private:
   
};
#endif //IPATHGENERATOR_HPP