#ifndef FIXEDHANG_H
#define FIXEDHANG_H

#include "FSM/FSMState.h"

class State_FixedHang : public FSMState
{
public:
    State_FixedHang(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
};

#endif // FIXEDHANG_H