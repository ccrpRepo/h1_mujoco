#ifndef STATE_DYNAMICTEST_H
#define STATE_DYNAMICTEST_H

#include "FSM/FSMState.h"

struct State_DynamicTest : public FSMState
{
public:
    State_DynamicTest(CtrlComponents* ctrlComp);
    ~State_DynamicTest(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    Eigen::Matrix<double, 19, 1> des_q;
    Dynamics *_dy;
};

#endif