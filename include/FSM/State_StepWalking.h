#ifndef STATE_STEPWALKING_H
#define STATE_STEPWALKING_H

#include "FSM/FSMState.h"
#include "control/WBCController.h"
#include "gait/GaitGenerator.h"

class State_StepWalking : public FSMState
{
public:
    State_StepWalking(CtrlComponents *ctrlComp);
    ~State_StepWalking();
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    virtual void getUserCmd();
    void calcCmd();
    void calcQQd();
    void calcTau();
    bool checkStepOrNot();
    void pin_init();

    GaitGenerator *_gait;
    Estimator *_est;
    h1Robot *_robot;

    Dynamics *_dy;
    WBC *_wbc;

    Eigen::Matrix<double, 3, 2> _posFeetGlobalInit;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    Mat3 _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec2 *_phase;
    VecInt2 *_contact;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal;
    Vec32 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec32 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec3 _ddPcd, _dWbd;
    Vec32 _forceFeetGlobal, _forceFeetBody;
    Mat52 _qGoal, _qdGoal;
    Eigen::Matrix<double, 19, 1> _tau;
    Eigen::Matrix<double, 19, 1> _tau_wbc, _tau_last;

    // Rob State
    Vec3 _posBody, _velBody;
    double _yaw, _dYaw;
    Vec32 _posFeetGlobal, _velFeetGlobal;
    Vec32 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Mat52 _q;
    Eigen::Matrix<double, 19, 1> _q_des, _qd_des;
};

#endif