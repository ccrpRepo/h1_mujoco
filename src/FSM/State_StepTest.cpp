#include "FSM/State_StepTest.h"

State_StepTest::State_StepTest(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::STEPTEST, "stepTest"),
      _est(ctrlComp->estimator), _robot(ctrlComp->_robot),
       _contact(ctrlComp->contact),
      _phase(ctrlComp->phase)
{
    _gaitHeight = 0.15;

    _KpSwing = Vec3(60, 60, 20).asDiagonal();
    _KdSwing = Vec3(20, 20, 5).asDiagonal();

    _Kpp = Vec3(50, 50, 300).asDiagonal();
    _Kpw = Vec3(600, 600, 600).asDiagonal();
    _Kdp = Vec3(5, 5, 20).asDiagonal();
    _Kdw = Vec3(10, 10, 10).asDiagonal();
}

void State_StepTest::enter()
{
    _ctrlComp->_robot->Update_Model();
    _pcd = _est->getPosition();
    _Rd = _lowState->getRotMat();
    _posFeetGlobalInit = _est->getFeetPos();
    _posFeetGlobalGoal = _posFeetGlobalInit;
    _ctrlComp->setStartWave();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_StepTest::run()
{
    // std::cout << "run" << std::endl;
    _ctrlComp->_robot->Update_Model();
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    for (int i(0); i < 2; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _posFeetGlobalGoal(2, i) = _posFeetGlobalInit(2, i) + (1 - cos((*_phase)(i) * 2 * M_PI)) * _gaitHeight;
            _velFeetGlobalGoal(2, i) = sin((*_phase)(i) * 2 * M_PI) * 2 * M_PI * _gaitHeight;
        }
    }
    _posFeetGlobalGoal(1, 0) = 0.2;
    _posFeetGlobalGoal(1, 1) = -0.2;
    Vec5 endposd_l, endposd_r;
    endposd_l.setZero();
    endposd_r.setZero();
    endposd_l.head(3) = _posFeetGlobalGoal.col(0);
    endposd_r.head(3) = _posFeetGlobalGoal.col(1);
    Eigen::Matrix<double, 19, 1> tar_pos;
    tar_pos.setZero();
    tar_pos.head(5) = _ctrlComp->_robot->getQ(endposd_l, 0);
    tar_pos.segment(5,5) = _ctrlComp->_robot->getQ(endposd_r, 1);
    // std::cout << "foot_des: " << endposd_l.transpose() << std::endl;
    _lowCmd->setSwingGain(0);
    _lowCmd->setSwingGain(1);
    _lowCmd->setQ(tar_pos);
    // std::cout << "pos: " << endposd_l.transpose() << std::endl;

    // std::cout << "des pos" << std::endl
    //           << _posFeetGlobalGoal << std::endl;
    // std::cout << "des vel" << std::endl
    //           << _velFeetGlobalGoal << std::endl;
    // calcTau();
    // _lowCmd->setZeroGain();
    // _lowCmd->setTau(_tau);
}

void State_StepTest::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_StepTest::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::STEPTEST;
    }
    else
    {
        return FSMStateName::STEPTEST;
    }
}

void State_StepTest::calcTau()
{
    _ddPcd = _Kpp * (_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd = _Kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    // _forceFeetGlobal = -_balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    _forceFeetGlobal.setZero();
    _posFeetGlobal = _est->getFeetPos();
    
    _velFeetGlobal = _est->getFeetVel();
    _KdSwing.setZero();
    for (int i(0); i < 2; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
        }
        else
        {
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
        }
    }
    
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    
    _q = _lowState->getQ();
    Mat3 Rl = _ctrlComp->_robot->T_foot[0].block(0, 0, 3, 3);
    Mat3 Rr = _ctrlComp->_robot->T_foot[1].block(0, 0, 3, 3);
    Eigen::Matrix<double, 6, 2> _forceFeetAnkel;
    _forceFeetAnkel.setZero();
    Rl.setIdentity();
    Rr.setIdentity();
    _forceFeetAnkel.col(0).tail(3) = Rl.transpose() * _forceFeetBody.col(0); // coordinate ankle
    _forceFeetAnkel.col(1).tail(3) = Rr.transpose() * _forceFeetBody.col(1); // coordinate ankle
    // std::cout << "_forceFeetAnkel" << std::endl
    //           << _forceFeetAnkel << std::endl;

    Mat6 X_fl, X_fr;
    X_fl.setIdentity();
    for (int i = 0; i < 5; i++)
    {
        X_fl = X_fl * _ctrlComp->_robot->X_dwtree[i];
    }
    for (int i = 5; i < 10; i++)
    {
        X_fr = X_fr * _ctrlComp->_robot->X_dwtree[i];
    }
    Eigen::Matrix<double, 6, 2> force;
    force.col(0) = X_fl.transpose().inverse() * _forceFeetAnkel.col(0);
    force.col(1) = X_fr.transpose().inverse() * _forceFeetAnkel.col(1);
    Eigen::Matrix<double, 6, 5> Jl = _ctrlComp->_robot->leg_Jacobian(_q.col(0), 0);
    Eigen::Matrix<double, 6, 5> Jr = _ctrlComp->_robot->leg_Jacobian(_q.col(1), 1);
    Eigen::Matrix<double, 19, 1> torque;
    torque.setZero();
    torque.segment(0, 5) = Jl.transpose() * force.col(0);
    torque.segment(5, 5) = Jr.transpose() * force.col(1);
    _lowCmd->setTau(torque);
}