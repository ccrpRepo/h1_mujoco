#ifndef STATE_SWINGTEST_H
#define STATE_SWINGTEST_H

#include "FSM/FSMState.h"
// #include "gait/GaitGenerator.h"

class State_SwingTest : public FSMState
{
public:
    State_SwingTest(CtrlComponents *ctrlComp);
    ~State_SwingTest(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    void _positionCtrl();
    void _torqueCtrl();

    Eigen::Matrix<double, 3, 2> _initFeetPos, _feetPos;
    Vec3 _initPos, _posGoal;
    Eigen::Matrix<double, 19, 1> _targetPos;
    float _xMin, _xMax;
    float _yMin, _yMax;
    float _zMin, _zMax;
    Eigen::Matrix<double,6,6> _Kp, _Kd;
};

#endif