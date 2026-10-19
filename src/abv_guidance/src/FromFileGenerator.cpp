
#include "abv_guidance/FromFileGenerator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <limits>
#include "plog/Log.h"

FromFileGenerator::FromFileGenerator() :
    mIndex(0), mPreviewLength(std::numeric_limits<std::size_t>::max())
{

}

FromFileGenerator::~FromFileGenerator()
{

}

bool FromFileGenerator::init()
{
    std::string filePath =
        ament_index_cpp::get_package_share_directory("abv_bringup") +
            "/config/path.csv";

    std::ifstream file(filePath);

    if (!file.is_open())
    {
        LOGW << "Failed to open path file from " + filePath;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }

        // Metadata/header line, e.g. "# PreviewLength: 5" - configures how
        // many upcoming waypoints getPathPreviewLength() advertises for
        // visualization. May appear anywhere in the file (not just the top);
        // unrecognized keys are logged and ignored. Everything else below
        // this block is unchanged waypoint-row parsing.
        if (line[0] == '#')
        {
            std::string content = line.substr(1);
            std::size_t colonPos = content.find(':');
            if (colonPos == std::string::npos)
            {
                LOGW << "Malformed path file header line (expected 'key: value'): " << line;
                continue;
            }

            std::string key = content.substr(0, colonPos);
            std::string value = content.substr(colonPos + 1);

            auto trim = [](std::string& s) {
                std::size_t start = s.find_first_not_of(" \t");
                std::size_t end = s.find_last_not_of(" \t");
                s = (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
            };
            trim(key);
            trim(value);

            if (key == "PreviewLength")
            {
                try
                {
                    mPreviewLength = std::stoul(value);
                }
                catch (const std::exception&)
                {
                    LOGW << "Invalid PreviewLength value in path file header: " << value;
                }
            }
            else
            {
                LOGW << "Unrecognized path file header key: " << key;
            }

            continue;
        }

        std::stringstream ss(line);
        std::string field;
        double x, y, yaw;
        double timeout = -1.0; // -1 means "use the configured default"
        Eigen::Vector3d arrivalTol(-1.0, -1.0, -1.0); // per-axis <= 0 means "no override"

        std::getline(ss, field, ','); x = std::stod(field);
        std::getline(ss, field, ','); y = std::stod(field);
        std::getline(ss, field, ','); yaw = std::stod(field);

        // optional 4th column overrides the default waypoint timeout
        if (std::getline(ss, field, ','))
        {
            timeout = std::stod(field);
        }

        // optional 5th-7th columns override the arrival tolerance [m, m, rad]
        // for this waypoint; all three must be present or none are applied
        std::string xTolField, yTolField, yawTolField;
        if (std::getline(ss, xTolField, ',') &&
            std::getline(ss, yTolField, ',') &&
            std::getline(ss, yawTolField, ','))
        {
            arrivalTol << std::stod(xTolField), std::stod(yTolField), std::stod(yawTolField);
        }

        LOGV << "Wp: " << x << "," << y << "," << yaw << " (timeout: " << timeout
             << ", tol: " << arrivalTol.transpose() << ")";
        mPath.emplace_back(x, y, yaw, "pose", timeout, arrivalTol);
    }

    mIndex = 0;
    return true;
}

bool FromFileGenerator::hasNext()
{
    return mIndex < mPath.size();
}

Waypoint FromFileGenerator::getNext()
{
    LOGV << "Sending waypoint " << mIndex;
    return mPath.at(mIndex++);
}

std::vector<Waypoint> FromFileGenerator::getPath() const
{
    return std::vector<Waypoint>(mPath.begin() + mIndex, mPath.end());
}

std::size_t FromFileGenerator::getPathPreviewLength() const
{
    // Configured via path.csv's "# PreviewLength: N" header line (see
    // init()); defaults to showing everything remaining. StateMachine clamps
    // this to the actual remaining count, so no upper-bound check is needed
    // here even for the default sentinel value.
    return mPreviewLength;
}
