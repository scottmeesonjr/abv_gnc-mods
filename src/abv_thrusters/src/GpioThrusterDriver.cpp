
#include "abv_thrusters/GpioThrusterDriver.h"
#include "plog/Log.h"
#include <algorithm>

GpioThrusterDriver::GpioThrusterDriver(std::vector<int> aSetOfOutputPins) : mOutputPins(aSetOfOutputPins)
{
    // do nothing
}

GpioThrusterDriver::~GpioThrusterDriver()
{
    writeAll(0);
    gpioTerminate();
}

bool GpioThrusterDriver::init()
{
    if(-1 == gpioInitialise())
    {
        LOGE << "Unable to initialize gpio";
        return false;
    }

    LOGD << "Gpio Initialized successfully!";

    for(const auto& pin : mOutputPins)
    {
        if(-1 == gpioSetMode(pin, JET_OUTPUT))
        {
            LOGE << "Unable to set pin: " << pin << "to mode: " << JET_OUTPUT;
            return false;
        }

        LOGD << "Set pin: " << pin << " Mode: " << JET_OUTPUT;
        writePin(pin, 0);
    }

    LOGD << "Initialized all output pins successfully";

    // Other setup stuff here
    return true;
}

bool GpioThrusterDriver::fini()
{
    gpioTerminate();
    return true;
}

bool GpioThrusterDriver::send(const std::string& aThrusterCommand)
{
    std::bitset<8> command(aThrusterCommand);
    writePins(command);
    return true;
}

bool GpioThrusterDriver::areAllPinsOff()
{
    for(const auto& pin : mOutputPins)
    {
        // TODO: i am assuming this returns 1 if the pin is high?
        if(1 == gpioRead(pin))
        {
            return false;
        }
    }

    return true;
}

void GpioThrusterDriver::writeAll(const int aState)
{
    for(const auto& pin : mOutputPins)
    {
        gpioWrite(pin, aState);         // skipping isOutputPin check since we are looping through mOutputPins
    }
}

void GpioThrusterDriver::writePin(int aPin, int aState)
{
    if(isOutputPin(aPin))
        gpioWrite(aPin, aState);
}

void GpioThrusterDriver::writePins(std::bitset<8> aThrustCommand)
{
    for(int i = 0; i < 8; i++)
    {
        int j = 7 - i;
        gpioWrite(mOutputPins[j], aThrustCommand[i]);       // skipping isOutputPin check since we are directly using mOutputPins
    }
}

bool GpioThrusterDriver::isOutputPin(int aPin)
{
    return std::find(mOutputPins.begin(), mOutputPins.end(), aPin) != mOutputPins.end();
}
