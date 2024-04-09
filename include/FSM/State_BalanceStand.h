#ifndef STATE_BALANCESTAND_H
#define STATE_BALANCESTAND_H

#include "FSM/FSMState.h"
#include "control/WBCController.h"

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
    Vec3 _init_pos;
    Mat3 _init_R_base;
    Estimator *_est;
    h1Robot *_robot;
    VecInt2 *_contact;
    Vec2 *_phase;

    RotMat _Rd;
    Vec3 _pcd;

    Dynamics *_dy;
    WBC *_wbc;

    Eigen::Matrix<double, 3, 2> _posFeetGlobalInit;
};

#endif