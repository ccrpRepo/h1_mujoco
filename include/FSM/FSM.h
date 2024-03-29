#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSM/FSMState.h"

#include "common/enumClass.h"
#include "control/CtrlComponents.h"
#include "FSM/State_Passive.h"
#include "FSM/State_SwingTest.h"

struct FSMStateList
{
    FSMState *invalid;
    State_Passive *passive;
    State_SwingTest *swingTest;

    // State_FixedStand *fixedStand;
    // State_FreeStand *freeStand;
    // State_Trotting *trotting;
    // State_BalanceTest *balanceTest;
    // State_SwingTest *swingTest;
    // State_StepTest *stepTest;

    void deletePtr()
    {
        delete invalid;
        delete passive;
        delete swingTest;
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