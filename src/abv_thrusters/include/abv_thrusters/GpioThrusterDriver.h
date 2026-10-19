#ifndef GPIOHANDLER_H
#define GPIOHANDLER_H   

#include "IThrusterDriver.hpp"

#include <jetgpio.h>
#include <stdexcept>
#include <unistd.h>
#include <vector> 
#include <array>
#include <bitset>

class GpioThrusterDriver : public IThrusterDriver
{
public:
    GpioThrusterDriver(std::vector<int> aSetOfOutputPins);
    ~GpioThrusterDriver() override; 

    bool init() override; 
    bool fini() override; 

    bool send(const std::string& aThrusterCommand) override; 

private:

    std::vector<int> mOutputPins; 

    bool isOutputPin(int aPin); 

    /**
     * @brief writeAll actuates all output pins (specified in mOutputPins) to the desired state
     * 
     * @param aState the state to actuate all output pins to. 1 = ON/HIGH, 0 = OFF/LOW
     */
    void writeAll(const int aState); 

    void writePin(int aPin, int aState); 
    
    void writePins(std::bitset<8>); 

    /**
     * @brief areAllPinsOff checks if all output pins (specified in mOutputPins) are off
     * 
     * @note this is a small optimization for when we arent receieving commands so we arent continuously
     *       trying to toggle pins off that are already off
     */
    bool areAllPinsOff();

};

#endif

