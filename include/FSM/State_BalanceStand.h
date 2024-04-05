#ifndef STATE_BALANCESTAND_H
#define STATE_BALANCESTAND_H

#include "FSM/FSMState.h"

class State_BalanceStand : public FSMState
{
public:
    State_BalanceStand(CtrlComponents *ctrlComp);
    ~State_BalanceStand(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    Dynamics *_dy;
    Eigen::Matrix<double, 25, 25> _H;
    Eigen::Matrix<double, 25, 1> _C;
    Eigen::Matrix<double, 10, 25> _K;
    Eigen::Matrix<double, 10, 1> _k;
};

#endif