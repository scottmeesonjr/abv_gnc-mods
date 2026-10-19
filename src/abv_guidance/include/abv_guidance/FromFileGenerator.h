#ifndef FROMFILEGENERATOR_H
#define FROMFILEGENERATOR_H
 
#include "abv_guidance/IPathGenerator.hpp"
 
class FromFileGenerator : public IPathGenerator
{ 
public:
    FromFileGenerator();
    ~FromFileGenerator() override; 

    bool init() override;
    bool hasNext() override;
    Waypoint getNext() override;
    std::vector<Waypoint> getPath() const override;
    std::size_t getPathPreviewLength() const override;

private:

    std::size_t mIndex;
    std::vector<Waypoint> mPath;

    // How many upcoming waypoints to advertise via getPathPreviewLength(),
    // configurable per-file via a "# PreviewLength: N" header line in
    // path.csv (see init()). Defaults to "show everything remaining".
    std::size_t mPreviewLength;

};
#endif //FROMFILEGENERATOR_H    