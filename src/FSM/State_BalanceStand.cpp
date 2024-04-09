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

    _ctrlComp->_d->qpos[0] = 1;
    _ctrlComp->_d->qpos[1] = 0;
    _ctrlComp->_d->qpos[2] = 0.98;
    _ctrlComp->_d->qpos[3] = 0.7071;
    _ctrlComp->_d->qpos[4] = 0;
    _ctrlComp->_d->qpos[5] = 0;
    _ctrlComp->_d->qpos[6] = 0.7071;

    _ctrlComp->_d->qpos[7] = 0;
    _ctrlComp->_d->qpos[8] = 0;
    _ctrlComp->_d->qpos[9] = -0.3;
    _ctrlComp->_d->qpos[10] = 0.8;
    _ctrlComp->_d->qpos[11] = -0.5;

    _ctrlComp->_d->qpos[12] = 0;
    _ctrlComp->_d->qpos[13] = 0;
    _ctrlComp->_d->qpos[14] = -0.3;
    _ctrlComp->_d->qpos[15] = 0.8;
    _ctrlComp->_d->qpos[16] = -0.5;

    _ctrlComp->_d->qpos[17] = 0;

    _ctrlComp->_d->qpos[18] = 0;
    _ctrlComp->_d->qpos[19] = 0;
    _ctrlComp->_d->qpos[20] = 0;
    _ctrlComp->_d->qpos[21] = 0;

    _ctrlComp->_d->qpos[22] = 0;
    _ctrlComp->_d->qpos[23] = 0;
    _ctrlComp->_d->qpos[24] = 0;
    _ctrlComp->_d->qpos[25] = 0;

    for (int i = 0; i < 25;i++)
    {
        _ctrlComp->_d->qvel[i] = 0;
        _ctrlComp->_d->qacc[i] = 0;
    }

    _init_pos = _ctrlComp->estimator->getPosition();
    _init_R_base = _ctrlComp->lowState->getRotMat();
    std::cout << "_init_pos: " << _init_pos.transpose() << std::endl;
}

void State_BalanceStand::run()
{
    _ctrlComp->_robot->Update_Model();
    _wbc->set_contact_frition(0.03);
    Mat3 R_base = _ctrlComp->lowState->imu.getRotMat();
    Mat4 T_base;
    T_base.setIdentity(4, 4);
    T_base.block(0, 0, 3, 3) = R_base.transpose() *_init_R_base;
    
    Mat4 T_tan = logm(T_base);
    Vec3 anglar_acc;
    anglar_acc << -T_tan(1, 2), T_tan(0, 2), -T_tan(0, 1);
    // std::cout << "ang: " << anglar_acc.transpose() << std::endl;
    Vec3 base_pos = _ctrlComp->estimator->getPosition();
    VecInt2 contact;
    contact << 1, 1;
    _wbc->dynamics_consistence_task(contact);
    _wbc->closure_constrain_task();
    Vec2 ddr_xy;
    Vec2 des_xy = _init_pos.head(2);
    ddr_xy = 20.0 * (des_xy - base_pos.head(2));
    // std::cout << "ddr_xy: " << ddr_xy.transpose() << std::endl;
    _wbc->desired_torso_motion_task(ddr_xy);
    Vec32 swing_acc;
    swing_acc.setZero();
    _wbc->swing_foot_motion_task(swing_acc, contact);
    double yaw_acc = 0, height_acc = 0;
    height_acc = 20 * (0.98 - base_pos(2));
    yaw_acc = 20 * anglar_acc(2); //
    _wbc->body_yaw_height_task(yaw_acc, height_acc);
    double roll_acc = 0, pitch_acc = 0;
    
    roll_acc = anglar_acc(0);
    pitch_acc = 20 * anglar_acc(1);
    // std::cout << "anglar_acc: " << anglar_acc.transpose() << std::endl;
    _wbc->body_roll_pitch_task(roll_acc, pitch_acc);
    _wbc->torque_limit_task();
    _wbc->friction_cone_task(contact);

    _wbc->solve_HOproblem();

    Eigen::Matrix<double, 19, 1> tau;
    tau = _wbc->_qdd_torque.block(25, 0, 19, 1);
    // tau.block(11, 0, 8, 1).setZero();
    // std::cout << "tau: " << tau.transpose() << std::endl;
    _lowCmd->setTau(tau);
    Eigen::Matrix<double, 19, 1> q_des, qd_des;
    q_des << 0, 0, -0.3, 0.8, -0.5,
        0, 0, -0.3, 0.8, -0.5,
        0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    _lowCmd->setQ(q_des);
    qd_des.setZero();
    _lowCmd->setQd(qd_des);
}

void State_BalanceStand::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_BalanceStand::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::BALANCESTAND;
    }
}