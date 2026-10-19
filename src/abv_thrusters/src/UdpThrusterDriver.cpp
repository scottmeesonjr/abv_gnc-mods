
#include "abv_thrusters/UdpThrusterDriver.h"

UdpThrusterDriver::UdpThrusterDriver(std::vector<int> aSet) : mClient("127.0.0.1", 6969)
{

}

UdpThrusterDriver::~UdpThrusterDriver()
{

}

bool UdpThrusterDriver::init()
{
    return true; 
}

bool UdpThrusterDriver::fini()
{
    return true; 
}

bool UdpThrusterDriver::send(const std::string& aThrusterCommand)
{
    return mClient.send(aThrusterCommand); 
}