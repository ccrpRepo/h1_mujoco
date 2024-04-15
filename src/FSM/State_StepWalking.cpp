#include "FSM/State_StepWalking.h"

State_StepWalking::State_StepWalking(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::STEPWALKING, "step walking"),
      _est(ctrlComp->estimator), _robot(ctrlComp->_robot),
      _contact(ctrlComp->contact),
      _phase(ctrlComp->phase)
{
    _wbc = _ctrlComp->_wbc;
    // _wbc = new WBC(_ctrlComp->dy);
    _dy = _ctrlComp->dy;
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.20;

    _Kpp = Vec3(50, 50, 20).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(40, 40, 40).asDiagonal();
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
    _wbc->set_contact_frition(0.45);
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
    // std::cout << *_phase << std::endl;
    calcQQd();
    calcTau();

    if (checkStepOrNot())
    {
        _ctrlComp->setStartWave();
    }
    else
    {
        _ctrlComp->setAllStance();
        // _ctrlComp->setStartWave();
    }

    _q_des.setZero();
    _qd_des.setZero();

    _q_des.block(0, 0, 5, 1) = _qGoal.col(0);
    _q_des.block(5, 0, 5, 1) = _qGoal.col(1);

    _qd_des.block(0, 0, 5, 1) = _qdGoal.col(0);
    _qd_des.block(5, 0, 5, 1) = _qdGoal.col(1);

    // std::cout << "qd_des: " << qd_des.transpose() << std::endl;
    if (_tau.array().isNaN().any())
    {
        _tau.setZero();
        std::cout << "_tau meets NaN" << std::endl;
    }
    if (_q_des.array().isNaN().any())
    {
        _q_des.setZero();
        std::cout << "q_des meets NaN" << std::endl;
    }
    if (_qd_des.array().isNaN().any())
    {
        _qd_des.setZero();
        std::cout << "qd_des meets NaN" << std::endl;
    }
    _tau.block(10, 0, 9, 1).setZero();
    // if((*_contact)(0) == 0)
    // {
    //     _tau.block(0, 0, 5, 1).setZero();
    // }
    // if ((*_contact)(1) == 0)
    // {
    //     _tau.block(5, 0, 5, 1).setZero();
    // }
    // std::cout << "_tau: " << _tau.transpose() << std::endl;
    // _tau.setZero();
    _lowCmd->setTau(_tau);
    // _lowCmd->setQ(_q_des);
    // _lowCmd->setQd(_qd_des);

    // _lowCmd->setWholeZeroGain();
    for (int i(0); i < 2; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _lowCmd->setSwingGain(i);
            _lowCmd->setArmGain();
            _lowCmd->setTorsoGain();
        }
        else
        {
            _lowCmd->setSimStanceGain(i);
            _lowCmd->setArmGain();
            _lowCmd->setTorsoGain();
            
        }
    }
    // _lowCmd->setSimStanceGain(0);
    // _lowCmd->setSimStanceGain(1);
    // _lowCmd->setArmGain();
    // _lowCmd->setTorsoGain();
    // _lowCmd->setWholeZeroGain();
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
    // std::cout << "_posFeetGlobalGoal_l: " << _posFeet2BGoal.col(0).transpose() << std::endl;
    for (int i(0); i < 2; ++i)
    {
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12)
    }
    // std::cout << "_posFeet2BGoal: " << _posFeetGlobalGoal.col(0).transpose() << std::endl;

    Vec5 _posFeet2BGoal_ext_l,
        _posFeet2BGoal_ext_r;
    _posFeet2BGoal_ext_l.setZero();
    _posFeet2BGoal_ext_r.setZero();
    _posFeet2BGoal_ext_l.head(3) = _posFeet2BGoal.col(0);
    _posFeet2BGoal_ext_r.head(3) = _posFeet2BGoal.col(1);

    // std::cout << "_posFeet2BGoal: " << _posFeet2BGoal.col(0).transpose() << std::endl;

    if (_posFeet2BGoal_ext_l(1) < -0.15)
        _posFeet2BGoal_ext_l(1) = -0.15;
    if (_posFeet2BGoal_ext_l(1) > 0.30)
        _posFeet2BGoal_ext_l(1) = 0.30;
    if (_posFeet2BGoal_ext_l(2) >-0.50)
        _posFeet2BGoal_ext_l(2) = -0.50;
    if (_posFeet2BGoal_ext_l(2) < -1)
        _posFeet2BGoal_ext_l(2) = -1;

    if (_posFeet2BGoal_ext_r(1) < -0.30)
        _posFeet2BGoal_ext_r(1) = -0.30;
    if (_posFeet2BGoal_ext_r(1) > 0.15)
        _posFeet2BGoal_ext_r(1) = 0.15;
    if (_posFeet2BGoal_ext_r(2) > -0.50)
        _posFeet2BGoal_ext_r(2) = -0.50;
    if (_posFeet2BGoal_ext_r(2) < -1)
        _posFeet2BGoal_ext_r(2) = -1;

    Vec5 q_des_l = _dy->Cal_inverse_kinematic_Analytical(_posFeet2BGoal_ext_l, 0);
    Vec5 q_des_r = _dy->Cal_inverse_kinematic_Analytical(_posFeet2BGoal_ext_r, 1);

    _qGoal.col(0) = q_des_l;
    _qGoal.col(1) = q_des_r;

    Eigen::Matrix<double, 3, 5> Jl = _ctrlComp->_robot->leg_Jacobian(_qGoal.col(0), 0).block(3, 0, 3, 5);
    Eigen::Matrix<double, 3, 5> Jr = _ctrlComp->_robot->leg_Jacobian(_qGoal.col(1), 1).block(3, 0, 3, 5);

    Eigen::Matrix<double, 5, 3> J_T;
    Eigen::Matrix<double, 3, 3> JJ_T;
    Eigen::Matrix<double, 3, 3> JJ_T_inv;
    J_T = Jl.transpose();
    JJ_T = Jl * J_T;
    // _velFeet2BGoal.col(0) << 0, 0, 1;
    // _velFeet2BGoal.col(1) << 0, 0, 1;
    JJ_T_inv = JJ_T.inverse();
    _qdGoal.col(0) = J_T * JJ_T_inv * _velFeet2BGoal.col(0);

    J_T = Jr.transpose();
    JJ_T = Jr * J_T;
    
    JJ_T_inv = JJ_T.inverse();
    _qdGoal.col(1) = J_T * JJ_T_inv * _velFeet2BGoal.col(1);
    if(_qdGoal.array().isNaN().any())
    {
        std::cout << "feetl: " << _posFeet2BGoal_ext_l.transpose() << std::endl;
        std::cout << "feetr: " << _posFeet2BGoal_ext_r.transpose() << std::endl;
        std::cout << _qGoal.transpose() << std::endl;
    }

    if ((*_contact)(0) == 0)
    {
        _ctrlComp->_d->qpos[0 + 7] = q_des_l(0);
        _ctrlComp->_d->qpos[1 + 7] = q_des_l(1);
        _ctrlComp->_d->qpos[2 + 7] = q_des_l(2);
        _ctrlComp->_d->qpos[3 + 7] = q_des_l(3);
        _ctrlComp->_d->qpos[4 + 7] = q_des_l(4);
    }
    if ((*_contact)(1) == 0)
    {
        _ctrlComp->_d->qpos[5 + 7] = q_des_r(0);
        _ctrlComp->_d->qpos[6 + 7] = q_des_r(1);
        _ctrlComp->_d->qpos[7 + 7] = q_des_r(2);
        _ctrlComp->_d->qpos[8 + 7] = q_des_r(3);
        _ctrlComp->_d->qpos[9 + 7] = q_des_r(4);
    }

    // if ((*_contact)(0) == 0)
    // {
    //     _ctrlComp->_d->qvel[0 + 7] = _qdGoal.col(0)(0);
    //     _ctrlComp->_d->qvel[1 + 7] = _qdGoal.col(0)(1);
    //     _ctrlComp->_d->qvel[2 + 7] = _qdGoal.col(0)(2);
    //     _ctrlComp->_d->qvel[3 + 7] = _qdGoal.col(0)(3);
    //     _ctrlComp->_d->qvel[4 + 7] = _qdGoal.col(0)(4);
    // }
    // if ((*_contact)(1) == 0)
    // {
    //     _ctrlComp->_d->qvel[5 + 7] = _qdGoal.col(1)(0);
    //     _ctrlComp->_d->qvel[6 + 7] = _qdGoal.col(1)(1);
    //     _ctrlComp->_d->qvel[7 + 7] = _qdGoal.col(1)(2);
    //     _ctrlComp->_d->qvel[8 + 7] = _qdGoal.col(1)(3);
    //     _ctrlComp->_d->qvel[9 + 7] = _qdGoal.col(1)(4);
    // }
    // _ctrlComp->_d->qpos[0] = 0;
    // _ctrlComp->_d->qpos[1] = 0;
    // _ctrlComp->_d->qpos[2] = 0.98;
    // _ctrlComp->_d->qpos[3] = 1;
    // _ctrlComp->_d->qpos[4] = 0;
    // _ctrlComp->_d->qpos[5] = 0;
    // _ctrlComp->_d->qpos[6] = 0;

}

void State_StepWalking::calcTau()
{
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;
    // std::cout << "_posError: " << _posError.transpose() << std::endl;
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
    // std::cout << ddp_base.transpose() << std::endl;
    // *_contact << 1, 0;
    _wbc->dynamics_consistence_task(*_contact);
    _wbc->closure_constrain_task();
    Vec2 ddr_xy;
    ddr_xy << 50 * ddp_base(0), 50 * ddp_base(1);
    // ddr_xy.setZero();
    _wbc->desired_torso_motion_task(ddr_xy);
    Vec32 swingforceFeetBase = _G2B_RotMat * _forceFeetGlobal;
    Vec3 swingforce;
    for (int i = 0; i < 2; i++)
    {
        if ((*_contact)(i) == 1)
        {
            swingforceFeetBase.col(i).setZero();
        }
        else
        {
            swingforce = swingforceFeetBase.col(i);
        }
    }
    // std::cout <<"phase: " <<(*_phase).transpose() << std::endl;
    // std::cout << swingforce.transpose() << std::endl;
    // swingforce.setZero();
    _wbc->swing_foot_motion_task(swingforce, *_contact, false);
    double yaw_acc = 30 * dw_base(2); 
    double height_acc = 30 *ddp_base(2);
    // std::cout << "height_acc: " << height_acc << std::endl;
    // height_acc = 100;
    _wbc->body_yaw_height_task(yaw_acc, height_acc);
    double roll_acc = 30 * dw_base(0);  //
    double pitch_acc = 50 * dw_base(1); //
    _wbc->body_roll_pitch_task(roll_acc, pitch_acc);
    _wbc->torque_limit_task(*_contact, true);
    _wbc->friction_cone_task(*_contact);
    _wbc->solve_HOproblem();

    _tau = _wbc->_qdd_torque.block(25, 0, 19, 1);

    // _ctrlComp->_d->qfrc_applied[0] = -5;
}
