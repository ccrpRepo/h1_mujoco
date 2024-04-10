#include "FSM/State_StepWalking.h"

State_StepWalking::State_StepWalking(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::STEPWALKING, "step walking"),
      _est(ctrlComp->estimator), _robot(ctrlComp->_robot),
      _contact(ctrlComp->contact),
      _phase(ctrlComp->phase)
{
    _wbc = _ctrlComp->_wbc;
    _dy = _ctrlComp->dy;
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.25;

    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();

    _vxLim = _robot->getRobVelLimitX();
    _vyLim = _robot->getRobVelLimitY();
    _wyawLim = _robot->getRobVelLimitYaw();
}

State_StepWalking::~State_StepWalking()
{
    delete _gait;
}

void State_StepWalking::enter()
{
    _pcd = _est->getPosition();
    _pcd(2) = -_robot->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

void State_StepWalking::run()
{
    _ctrlComp->_robot->Update_Model();
    _wbc->set_contact_frition(0.35);
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();
    
    _gait->setGait(_vCmdGlobal.segment(0, 2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcQQd();
    calcTau();

    // if (checkStepOrNot())
    // {
    //     _ctrlComp->setStartWave();
    // }
    // else
    // {
    //     // _ctrlComp->setAllStance();
    //     _ctrlComp->setStartWave();
    // }

    // Eigen::Matrix<double, 19, 1> q_des, qd_des;
    // q_des.setZero();
    // qd_des.setZero();

    // q_des.block(0, 0, 5, 1) = _qGoal.col(0);
    // q_des.block(5, 0, 5, 1) = _qGoal.col(1);

    // qd_des.block(0, 0, 5, 1) = _qdGoal.col(0);
    // qd_des.block(5, 0, 5, 1) = _qdGoal.col(1);

    // _lowCmd->setTau(_tau);
    // _lowCmd->setQ(q_des);
    // _lowCmd->setQd(qd_des);

    // for (int i(0); i < 2; ++i)
    // {
    //     if ((*_contact)(i) == 0)
    //     {
    //         _lowCmd->setSwingGain(i);
    //     }
    //     else
    //     {
    //         _lowCmd->setStableGain(i);
    //     }
    // }
}

bool State_StepWalking::checkStepOrNot()
{
    if ((fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20))
    {
        return true;
    }
    else
    {
        return false;
        // return true; //
    }
}

void State_StepWalking::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}


FSMStateName State_StepWalking::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::STEPWALKING;
    }
}

void State_StepWalking::getUserCmd()
{
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_StepWalking::calcCmd()
{
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.2, _velBody(0) + 0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.2, _velBody(1) + 0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

void State_StepWalking::calcQQd()
{
    Vec32 _posFeet2B;
    _posFeet2B = _robot->get_footEnd().block(0, 0, 3, 2);

    for (int i(0); i < 2; ++i)
    {
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12)
    }

    Vec5 _posFeet2BGoal_ext_l, _posFeet2BGoal_ext_r;
    _posFeet2BGoal_ext_l.setZero();
    _posFeet2BGoal_ext_r.setZero();
    _posFeet2BGoal_ext_l.head(3) = _posFeet2BGoal.col(0);
    _posFeet2BGoal_ext_r.head(3) = _posFeet2BGoal.col(1);

    Vec5 q_des_l = _dy->Cal_inverse_kinematic_Analytical(_posFeet2BGoal_ext_l, 0);
    Vec5 q_des_r = _dy->Cal_inverse_kinematic_Analytical(_posFeet2BGoal_ext_r, 1);
    _qGoal.col(0) = q_des_l;
    _qGoal.col(1) = q_des_r;

    Eigen::Matrix<double, 6, 2> velFeetAnkel;
    velFeetAnkel.setZero();
    Mat3 Rl = _ctrlComp->_robot->T_base.block(0, 0, 3, 3) * _ctrlComp->_robot->T_foot[0].block(0, 0, 3, 3);
    Mat3 Rr = _ctrlComp->_robot->T_base.block(0, 0, 3, 3) * _ctrlComp->_robot->T_foot[1].block(0, 0, 3, 3);
    velFeetAnkel.col(0).tail(3) = Rl.transpose() * _velFeet2BGoal.col(0); // coordinate ankle
    velFeetAnkel.col(1).tail(3) = Rr.transpose() * _velFeet2BGoal.col(1); // coordinate ankle
    Mat6 X_fl, X_fr;
    X_fl.setIdentity();
    X_fr.setIdentity();
    for (int i = 0; i < 5; i++)
    {
        X_fl = X_fl * _ctrlComp->_robot->X_dwtree[i];
    }
    for (int i = 5; i < 10; i++)
    {
        X_fr = X_fr * _ctrlComp->_robot->X_dwtree[i];
    }

    Eigen::Matrix<double, 6, 2> xvel;
    xvel.col(0) = X_fl * velFeetAnkel.col(0); // coordinate base
    xvel.col(1) = X_fr * velFeetAnkel.col(1);

    Eigen::Matrix<double, 6, 5> Jl = _ctrlComp->_robot->leg_Jacobian(_qGoal.col(0), 0);
    Eigen::Matrix<double, 6, 5> Jr = _ctrlComp->_robot->leg_Jacobian(_qGoal.col(1), 1);

    Eigen::Matrix<double, 5, 6> J_T;
    Eigen::Matrix<double, 5, 5> J_TJ;
    Eigen::Matrix<double, 5, 6> J_pesuinv;
    J_T = Jl.transpose();
    J_TJ = J_T * Jl;
    J_pesuinv = J_TJ.inverse() * J_T;

    _qdGoal.col(0) = J_pesuinv * xvel.col(0);

    J_T = Jr.transpose();
    J_TJ = J_T * Jr;
    J_pesuinv = J_TJ.inverse() * J_T;
    _qdGoal.col(1) = J_pesuinv * xvel.col(1);


}

void State_StepWalking::calcTau()
{
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal.setZero();
    for (int i(0); i < 2; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
            // std::cout << "err   " << _posFeetGlobalGoal.col(i).transpose() << std::endl;
        }
    }

    Vec3 dw_base = _G2B_RotMat * _dWbd;
    Vec3 ddp_base = _G2B_RotMat * _ddPcd;
    _wbc->dynamics_consistence_task(*_contact);
    _wbc->closure_constrain_task();
    Vec2 ddr_xy;
    ddr_xy << ddp_base(0) * 0.3, ddp_base(1) * 1.5;
    _wbc->desired_torso_motion_task(ddr_xy);
    Vec32 swingforceFeetBase = _G2B_RotMat * _forceFeetGlobal;
    
    for (int i = 0; i < 2; i++)
    {
        if ((*_contact)(i) == 1)
        {
            swingforceFeetBase.col(i).setZero();
        }
    }

    _wbc->swing_foot_motion_task(swingforceFeetBase, *_contact);
    double yaw_acc = dw_base(2) * 1.4;     //
    double height_acc = ddp_base(2) * 1.0; //
    _wbc->body_yaw_height_task(yaw_acc, height_acc);
    double roll_acc = dw_base(0) * 15.0;  //
    double pitch_acc = dw_base(1) * 30.0; //
    _wbc->body_roll_pitch_task(roll_acc, pitch_acc);
    _wbc->torque_limit_task();
    _wbc->friction_cone_task(*_contact);
    // long long start_time = getSystemTime();
    _wbc->solve_HOproblem();

    _tau = _wbc->_qdd_torque.block(25, 0, 19, 1);
}
