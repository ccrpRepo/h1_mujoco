#include "FSM/State_BalanceStand.h"


State_BalanceStand::State_BalanceStand(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::BALANCESTAND, "balance stand"),
      _est(ctrlComp->estimator), _robot(ctrlComp->_robot),
      _contact(ctrlComp->contact),
      _phase(ctrlComp->phase) 
    {
        _wbc = _ctrlComp->_wbc;
    }

void State_BalanceStand::enter()
{
    _ctrlComp->_robot->Update_Model();
    _pcd = _est->getPosition();
    _Rd = _lowState->getRotMat();
    _posFeetGlobalInit = _est->getFeetPos();
    _ctrlComp->setStartWave();
    _ctrlComp->ioInter->zeroCmdPanel();
    
    _lowCmd->setSimStanceGain(0);
    _lowCmd->setSimStanceGain(1);
    _lowCmd->setArmGain();
    _lowCmd->setTorsoGain();
    // _lowCmd->motorCmd[1].Kp = 1000;
    // _lowCmd->motorCmd[6].Kp = 1000;
    // _lowCmd->setZeroGain(0);
    // _lowCmd->setZeroGain(1);
    // _lowCmd->setWholeZeroGain();
    // _lowCmd->setWholeSmallGain();

    _q_des << 0, 0, -0.3, 1.0, -0.7,
        0, 0, -0.3, 1.0, -0.7,
        0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    _qd_des.setZero();

    Vec3 init_pos(0, 0, 0.98);
    Quat init_quat;
    init_quat << 1, 0, 0, 0;
    Mat3 init_R = quatToRotMat(init_quat);

    _ctrlComp->_d->qpos[0] = init_pos(0);
    _ctrlComp->_d->qpos[1] = init_pos(1);
    _ctrlComp->_d->qpos[2] = init_pos(2);
    _ctrlComp->_d->qpos[3] = init_quat(0);
    _ctrlComp->_d->qpos[4] = init_quat(1);
    _ctrlComp->_d->qpos[5] = init_quat(2);
    _ctrlComp->_d->qpos[6] = init_quat(3);

    for (int i = 0; i < 19;i++)
    {
        _ctrlComp->_d->qpos[i+7] = _q_des(i);
    }
    
    for (int i = 0; i < 25;i++)
    {
        _ctrlComp->_d->qvel[i] = 0;
        _ctrlComp->_d->qacc[i] = 0;
    }

    _init_pos = init_pos;
    _init_R_base = init_R;
}

void State_BalanceStand::run()
{
    _ctrlComp->_robot->Update_Model();

    _wbc->set_contact_frition(0.35);
    Mat3 R_base = _ctrlComp->lowState->imu.getRotMat();
    Mat4 T_base;
    T_base.setIdentity(4, 4);
    T_base.block(0, 0, 3, 3) = _init_R_base * R_base.transpose();

    Mat4 T_tan = logm(T_base);
    Vec3 anglar_acc;
    anglar_acc << -T_tan(1, 2), T_tan(0, 2), -T_tan(0, 1);
    anglar_acc = _ctrlComp->lowState->getRotMat().transpose() * anglar_acc;
    // std::cout << "ang: " << anglar_acc.transpose() << std::endl;
    Vec3 base_pos = _ctrlComp->estimator->getPosition();
    VecInt2 contact;
    contact << 1, 1;
    _wbc->dynamics_consistence_task(contact);
    _wbc->closure_constrain_task();
    // _init_pos(2) = 0.7;
    // std::cout << "base_pos: " << base_pos.transpose() << std::endl;
    Vec3 pos_err = _init_pos - base_pos;
    pos_err = _ctrlComp->lowState->getRotMat().transpose() * pos_err;
    // std::cout << "_posError: " << pos_err.transpose() << std::endl;
    Vec2 ddr_xy;
    Vec2 des_xy = _init_pos.head(2);
    ddr_xy = 50.0 * pos_err.head(2);
    
    _wbc->desired_torso_motion_task(ddr_xy);
    Vec3 swing_acc;
    swing_acc.setZero();
    _wbc->swing_foot_motion_task(swing_acc, contact, false);
    double yaw_acc = 0, height_acc = 0;
    height_acc = 30 * pos_err(2); // 20 * pos_err(2)
    yaw_acc = 20 * anglar_acc(2); //
    // std::cout << "height_acc: " << height_acc << std::endl;
    _wbc->body_yaw_height_task(yaw_acc, height_acc);
    double roll_acc = 0, pitch_acc = 0;
    
    roll_acc = 50 * anglar_acc(0);
    // std::cout << "roll:" << anglar_acc(0) << std::endl;
    pitch_acc = 75 * anglar_acc(1);
    // std::cout << "pitch:" << anglar_acc(1) << std::endl;
    // std::cout << "anglar_acc: " << anglar_acc.transpose() << std::endl;
    _wbc->body_roll_pitch_task(roll_acc, pitch_acc);
    _wbc->torque_limit_task(contact,true);
    _wbc->friction_cone_task(contact);

    _wbc->solve_HOproblem();

    Eigen::Matrix<double, 19, 1> tau;
    tau = _wbc->_qdd_torque.block(25, 0, 19, 1);
    // tau.block(11, 0, 8, 1).setZero();
    // std::cout << "_tau: " << tau.transpose() << std::endl;
    _lowCmd->setTau(tau);

    _lowCmd->setQ(_q_des);
    
    _lowCmd->setQd(_qd_des);
    // _ctrlComp->_d->qpos[0] = 0;
    // _ctrlComp->_d->qpos[1] = 0;
    // _ctrlComp->_d->qpos[3] = 1;
    // _ctrlComp->_d->qpos[4] = 0;
    // _ctrlComp->_d->qpos[5] = 0;
    // _ctrlComp->_d->qpos[6] = 0;
    // _ctrlComp->_d->qfrc_applied[3] = 20;
}

void State_BalanceStand::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllStance();
}

FSMStateName State_BalanceStand::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::START)
    {
        return FSMStateName::STEPWALKING;
    }
    
    else
    {
        return FSMStateName::BALANCESTAND;
    }
}