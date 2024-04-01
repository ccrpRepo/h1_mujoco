#ifndef STEPTEST_H
#define STEPTEST_H

#include "FSM/FSMState.h"

class State_StepTest : public FSMState
{
public:
    State_StepTest(CtrlComponents *ctrlComp);
    
    ~State_StepTest() {}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    void calcTau();

    float _gaitHeight;

    Estimator *_est;
    h1Robot *_robot;
    // QuadrupedRobot *_robModel;
    // BalanceCtrl *_balCtrl;

    VecInt2 *_contact;
    Vec2 *_phase;

    RotMat _Rd;
    Vec3 _pcd;
    Mat3 _Kpp, _Kpw, _Kdp, _Kdw;
    Mat3 _KpSwing, _KdSwing;
    Vec3 _ddPcd, _dWbd;

    Mat52 _q, _tau;
    Vec3 _posBody, _velBody;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Eigen::Matrix<double, 3, 2> _posFeet2BGlobal;
    Eigen::Matrix<double, 3, 2> _posFeetGlobalInit, _posFeetGlobalGoal, _velFeetGlobalGoal;
    Eigen::Matrix<double, 3, 2> _posFeetGlobal, _velFeetGlobal;
    Eigen::Matrix<double, 3, 2> _forceFeetGlobal, _forceFeetBody;
};

#endif