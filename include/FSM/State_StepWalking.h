#ifndef STATE_STEPWALKING_H
#define STATE_STEPWALKING_H

#include "FSM/FSMState.h"
#include "control/WBCController.h"

class State_StepWalking : public FSMState
{
public:
    State_StepWalking(CtrlComponents *ctrlComp);
    ~State_StepWalking(){};
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

    Eigen::Matrix<double, 3, 2> _posFeetGlobalInit;
};

#endif