#include "FSM/State_SwingTest.h"

State_SwingTest::State_SwingTest(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::SWINGTEST, "swingTest")
{
    _xMin = -0.35;
    _xMax = 0.35;
    _yMin = -0.25;
    _yMax = 0.25;
    _zMin = -0.05;
    _zMax = 0.60;
}

void State_SwingTest::enter()
{
    for (int i = 0; i < 2; i++)
    {
        if (_ctrlComp->ctrlPlatform == CtrlPlatform::MUJOCO)
        {
            _lowCmd->setSimStanceGain(i);
        }
        else if (_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)
        {
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    _lowCmd->setSwingGain(0);

    Eigen::Matrix<double, 6, 1> Kp;
    Kp << 20, 100, 50, 50, 50, 50;
    Eigen::Matrix<double, 6, 1> Kd;
    Kd << 5, 5, 20, 10, 10, 10;
    _Kp = Kp.asDiagonal();
    _Kd = Kd.asDiagonal();

    for (int i = 0; i < 19; i++)
    {
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
    }
    _initFeetPos = _ctrlComp->_robot->get_footEnd().block(0, 0, 3, 2);
    _feetPos = _initFeetPos;
    _initPos = _initFeetPos.col(0);

    _ctrlComp->setAllSwing();
}

void State_SwingTest::run()
{
    _userValue = _lowState->userValue;

    if (_userValue.ly > 0)
    {
        _posGoal(0) = invNormalize(_userValue.ly, _initPos(0), _initPos(0) + _xMax, 0, 1);
    }
    else
    {
        _posGoal(0) = invNormalize(_userValue.ly, _initPos(0) + _xMin, _initPos(0), -1, 0);
    }

    if (_userValue.lx > 0)
    {
        _posGoal(1) = invNormalize(_userValue.lx, _initPos(1), _initPos(1) + _yMax, 0, 1);
    }
    else
    {
        _posGoal(1) = invNormalize(_userValue.lx, _initPos(1) + _yMin, _initPos(1), -1, 0);
    }

    if (_userValue.ry > 0)
    {
        _posGoal(2) = invNormalize(_userValue.ry, _initPos(2), _initPos(2) + _zMax, 0, 1);
    }
    else
    {
        _posGoal(2) = invNormalize(_userValue.ry, _initPos(2) + _zMin, _initPos(2), -1, 0);
    }
    // std::cout <<"posGoal: " <<_posGoal.transpose() << std::endl;
   
        
    _positionCtrl();
    _torqueCtrl();
}

void State_SwingTest::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_SwingTest::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        // std::cout << "change" << std::endl;
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        // std::cout << "change" << std::endl;
        return FSMStateName::FIXEDSTAND;
        
    }
    else
    {
        // std::cout << "not change" << std::endl;
        return FSMStateName::SWINGTEST;
    }
}

void State_SwingTest::_positionCtrl()
{
    _feetPos.col(0) = _posGoal;
    Vec5 endposd_l;
    endposd_l.setZero();
    endposd_l.head(3) = _feetPos.col(0);
    // std::cout <<"des_pos: " <<_posGoal.transpose() << std::endl;
    _targetPos.setZero(19, 1);
    _targetPos.head(5) = _ctrlComp->_robot->getQ(endposd_l, 0);
    // std::cout << _targetPos.segment(1,3).transpose() << std::endl;
    _lowCmd->setQ(_targetPos);
}

void State_SwingTest::_torqueCtrl()
{

    Eigen::Matrix<double,5,2> footend = _ctrlComp->_robot->get_footEnd();
    Vec3 pos0 = footend.col(0).head(3);
    Vec6 vel0 = _ctrlComp->_robot->getFootVelocity(0);
    Vec6 pos_error;
    pos_error.setZero();
    pos_error.tail(3) = _posGoal - pos0;
    // std::cout << "pos0: " << pos0.transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> force0 = _Kp * (pos_error);
    Mat6 X_f;
    X_f.setIdentity();
    for (int i = 0; i < 5; i++)
    {
        X_f = X_f * _ctrlComp->_robot->X_dwtree[i];
    }
    force0 = X_f.transpose().inverse() * force0;
    Eigen::Matrix<double, 19, 1>  torque;
    Vec5 q;
    for (int i = 0; i < 5; i++)
    {
        q(i) = _ctrlComp->_robot->_q[i];
    }
    Eigen::Matrix<double, 6, 5> J = _ctrlComp->_robot->leg_Jacobian(q, 0);
    torque.segment(0, 5) = J.transpose() * force0;
    _lowCmd->setTau(torque);
}