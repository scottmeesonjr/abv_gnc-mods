#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <memory>
#include <thread>
#include <mutex>

#include "abv_goal_manager/GoalManager.h"

class StateMachine
{
public:
    StateMachine(std::shared_ptr<GoalManager> abv);
    ~StateMachine();

    enum class States
    {
        STARTUP,
        IDLE,
        THRUSTER_CONTROL,
        DIRECTION_CONTROL,
        POSE_CONTROL,
        VELOCITY_CONTROL,
        NAV_FAULT,
        NUM_TYPES
    };

    void run(); 

    std::string toString(States aState); 

    /** Getters and Setters **/
    bool isDone() {std::lock_guard<std::mutex> lock(mDoneMutex); return mDone; }
    void stop() {std::lock_guard<std::mutex> lock(mDoneMutex); mDone = true; }
    void setActiveState(States aNewActiveState); 
    States getActiveState() {std::lock_guard<std::mutex> lock(mActiveStateMutex); return mActiveState;}

private:
    bool mDone;
    std::mutex mDoneMutex;
    States mActiveState;
    std::mutex mActiveStateMutex;
    std::shared_ptr<GoalManager> mGoalManager;

    // state to return to once nav data recovers from a NAV_FAULT; only set
    // for POSE_CONTROL/VELOCITY_CONTROL so a fault from DIRECTION_CONTROL
    // (a raw, non-persistent teleop input) resolves back to IDLE instead
    States mPreFaultState{States::IDLE};

    std::thread mControlStatusPublishThread;

private: 
    void controlStatusPublishLoop(); 

};
#endif // STATEMACHINE_H
