
#include "abv_guidance/FromFileGenerator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>
#include "plog/Log.h"

#include "abv_msgs/msg/dock_command.hpp"
#include "abv_common/ConfigurationManager.h"

namespace
{
    std::string trimmed(const std::string& s)
    {
        std::size_t start = s.find_first_not_of(" \t\r");
        std::size_t end = s.find_last_not_of(" \t\r");
        return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
    }

    std::vector<std::string> splitCsv(const std::string& aLine)
    {
        std::vector<std::string> fields;
        std::stringstream ss(aLine);
        std::string field;
        while (std::getline(ss, field, ','))
        {
            fields.push_back(trimmed(field));
        }
        return fields;
    }

    bool isNumber(const std::string& s)
    {
        if (s.empty()) return false;
        char* end = nullptr;
        std::strtod(s.c_str(), &end);
        return end != s.c_str() && *end == '\0';
    }

    std::string lowercase(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
        return s;
    }

    // "x,y,yaw[,timeout][,tolX,tolY,tolYaw]" - the original row format
    Waypoint parsePose(const std::vector<std::string>& f, std::size_t i0)
    {
        if (f.size() < i0 + 3)
        {
            throw std::runtime_error("pose row needs x,y,yaw");
        }
        double timeout = -1.0; // -1 means "use the configured default"
        Eigen::Vector3d arrivalTol(-1.0, -1.0, -1.0); // per-axis <= 0 means "no override"

        // optional 4th column overrides the default waypoint timeout
        if (f.size() >= i0 + 4 && !f[i0 + 3].empty())
        {
            timeout = std::stod(f[i0 + 3]);
        }
        // optional 5th-7th columns override the arrival tolerance [m, m, rad]
        // for this waypoint; all three must be present or none are applied
        if (f.size() >= i0 + 7)
        {
            arrivalTol << std::stod(f[i0 + 4]), std::stod(f[i0 + 5]), std::stod(f[i0 + 6]);
        }
        return Waypoint(std::stod(f[i0]), std::stod(f[i0 + 1]), std::stod(f[i0 + 2]),
                        WaypointType::POSE, timeout, arrivalTol);
    }
}

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

    const DockingConfig& dockConfig = ConfigurationManager::getInstance()->getDockingConfig();

    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line))
    {
        lineNumber++;
        line = trimmed(line);
        if (line.empty())
        {
            continue;
        }

        // Metadata/header line, e.g. "# PreviewLength: 5" - configures how
        // many upcoming waypoints getPathPreviewLength() advertises for
        // visualization. May appear anywhere in the file (not just the top);
        // unrecognized keys are logged and ignored.
        if (line[0] == '#')
        {
            std::string content = line.substr(1);
            std::size_t colonPos = content.find(':');
            if (colonPos == std::string::npos)
            {
                LOGW << "Malformed path file header line (expected 'key: value'): " << line;
                continue;
            }

            std::string key = trimmed(content.substr(0, colonPos));
            std::string value = trimmed(content.substr(colonPos + 1));

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

        // Row formats (first column decides):
        //   x,y,yaw[,timeout][,tolX,tolY,tolYaw]        pose (original format)
        //   pose,x,y,yaw[,timeout][,tolX,tolY,tolYaw]   same, explicit
        //   vel,vx,vy,wz,duration                       velocity in table frame for duration s
        //   approach[,timeout]                          docking approach from config.yaml Docking block
        //   hold,duration                               stop thrusting, wait duration s
        //   arm | disarm | lock | unlock                command the docking mechanism
        std::vector<std::string> f = splitCsv(line);
        try
        {
            Waypoint wp;
            if (isNumber(f[0]))
            {
                wp = parsePose(f, 0);
            }
            else
            {
                std::string kind = lowercase(f[0]);
                if (kind == "pose")
                {
                    wp = parsePose(f, 1);
                }
                else if (kind == "vel")
                {
                    if (f.size() < 5)
                    {
                        throw std::runtime_error("vel row needs vx,vy,wz,duration");
                    }
                    wp = Waypoint::velocity(std::stod(f[1]), std::stod(f[2]), std::stod(f[3]), std::stod(f[4]));
                }
                else if (kind == "approach")
                {
                    double timeout = (f.size() >= 2 && !f[1].empty()) ? std::stod(f[1]) : dockConfig.mApproachTimeout;
                    auto v = DockingGeometry::approachVelocity(dockConfig, dockConfig.mApproachVelocity);
                    auto contact = DockingGeometry::chaserPoseAtGap(dockConfig, 0.0);
                    wp = Waypoint::approach(v[0], v[1], timeout,
                                            Eigen::Vector3d(contact[0], contact[1], contact[2]));
                }
                else if (kind == "hold")
                {
                    if (f.size() < 2)
                    {
                        throw std::runtime_error("hold row needs duration");
                    }
                    wp = Waypoint::hold(std::stod(f[1]));
                }
                else if (kind == "arm")    { wp = Waypoint::dockCommand(abv_msgs::msg::DockCommand::CMD_ARM); }
                else if (kind == "disarm") { wp = Waypoint::dockCommand(abv_msgs::msg::DockCommand::CMD_DISARM); }
                else if (kind == "lock")   { wp = Waypoint::dockCommand(abv_msgs::msg::DockCommand::CMD_LOCK); }
                else if (kind == "unlock") { wp = Waypoint::dockCommand(abv_msgs::msg::DockCommand::CMD_UNLOCK); }
                else
                {
                    throw std::runtime_error("unknown row type '" + f[0] + "'");
                }
            }

            LOGV << "Wp " << mPath.size() << ": " << wp.mType << " pose " << wp.mPose.transpose()
                 << " vel " << wp.mVel.transpose() << " (timeout: " << wp.mTimeout
                 << ", tol: " << wp.mArrivalTol.transpose() << ")";
            mPath.push_back(wp);
        }
        catch (const std::exception& e)
        {
            LOGE << "path.csv line " << lineNumber << " (" << line << "): " << e.what();
            mPath.clear();
            return false;
        }
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
