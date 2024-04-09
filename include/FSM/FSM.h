#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSM/FSMState.h"

#include "common/enumClass.h"
#include "control/CtrlComponents.h"
#include "FSM/State_Passive.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include "FSM/State_FixedHang.h"
#include "FSM/State_DynamicTest.h"
#include "FSM/State_BalanceStand.h"

struct FSMStateList
{
    FSMState *invalid;
    State_Passive *passive;
    State_SwingTest *swingTest;
    State_StepTest *stepTest;
    State_FixedHang *fixedHang;
    State_DynamicTest *dynamicTest;
    State_BalanceStand *balanceStand;
    // State_FixedStand *fixedStand;
    // State_FreeStand *freeStand;
    // State_Trotting *trotting;
    // State_BalanceTest *balanceTest;

    void
    deletePtr()
    {
        delete invalid;
        delete passive;
        delete swingTest;
        delete stepTest;
        delete fixedHang;
        delete dynamicTest;
        delete balanceStand;
    }
};

class FSM
{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
    

private:
    FSMState *getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};

#endif