#ifndef STATE_DYNAMICTEST_H
#define STATE_DYNAMICTEST_H

#include "FSM/FSMState.h"

class State_DynamicTest : public FSMState
{
public:
    State_DynamicTest(CtrlComponents *ctrlComp);
    ~State_DynamicTest(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    Dynamics *_dy;
};

#endif