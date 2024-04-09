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

    Estimator *_est;
    h1Robot *_robot;
    VecInt2 *_contact;
    Vec2 *_phase;

    RotMat _Rd;
    Vec3 _pcd;

    Dynamics *_dy;
    WBC *_wbc;
    Eigen::Matrix<double, 25, 25> _H;
    Eigen::Matrix<double, 25, 1> _C;
    Eigen::Matrix<double, 10, 25> _K;
    Eigen::Matrix<double, 10, 1> _k;

    Eigen::Matrix<double, 3, 2> _posFeetGlobalInit;
};

#endif