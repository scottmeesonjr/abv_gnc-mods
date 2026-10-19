#ifndef UDPTHRUSTERDRIVER_H
#define UDPTHRUSTERDRIVER_H

#include "IThrusterDriver.hpp"
#include "abv_common/UdpClient.h"
#include <vector> 

class UdpThrusterDriver : public IThrusterDriver
{
public: 
    UdpThrusterDriver(std::vector<int> aSet); 
    ~UdpThrusterDriver(); 

    bool init() override; 
    bool fini() override; 

    bool send(const std::string& aThrusterCommand) override;
private: 

    UdpClient mClient; 
};
#endif 