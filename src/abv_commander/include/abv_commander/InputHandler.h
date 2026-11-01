#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include "abv_commander/Colors.hpp"
#include <cstdint>
#include <string> 
 
class InputHandler 
{ 
public:
    InputHandler();
    ~InputHandler();

    void handle(const std::string& anInput); 

private:

    // Prompt for a number; blank input returns aDefault. Returns false (and
    // prints an error) if the input isn't a number.
    static bool promptNumber(const std::string& aPrompt, double aDefault, double& aOut);

    // Asks for radial offset, angular offset, approach speed and duration,
    // then sends a "dock" guidance command (see CyclicDockGenerator.h for how
    // the goal fields are used).
    void sendDockRun();
    void sendDockCommand(std::uint8_t aCode, const std::string& aName);

    std::string mPackagePath; 
   
};
#endif //INPUTHANDLER_H
