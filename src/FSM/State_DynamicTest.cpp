#include "FSM/State_DynamicTest.h"

State_DynamicTest::State_DynamicTest(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::DYNAMICTEST, "dynamicTest")
{
    _dy = _ctrlComp->dy;
}

void State_DynamicTest::enter()
{
    if (_ctrlComp->ctrlPlatform == CtrlPlatform::MUJOCO)
    {
        // _lowCmd->setWholeSmallGain();
        _lowCmd->setWholeZeroGain();
    }
    else if (_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)
    {
        // _lowCmd->setRealStanceGain(i);
    }

    Eigen::Matrix<double, 19, 1> des_q;
    des_q << 0, 0, -0.2, 0.4, 0.3,
        0, 0, -0.2, 0.4, 0.3,
        0,
        1, 1, 0, 0,
        1, -1, 0, 0;

    for (int i = 0; i < 19; i++)
    {
        _ctrlComp->_d->qpos[i] = des_q(i);
    }
}

void State_DynamicTest::run()
{
    _ctrlComp->_robot->Update_Model();

    Eigen::Matrix<double, 19, 1> tau;
    MatX Hfl,K,k;
    MatX H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(Hfl);
    K = _dy->Cal_K_Flt(k);
    tau = _dy->Cal_Generalize_Bias_force_Flt(true).block(6, 0, 19, 1);

    _lowCmd->setTau(tau);
    
    // _lowCmd->setQ(des_q);

    // _ctrlComp->_d->qfrc_applied[1] = 1;
}

void State_DynamicTest::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_DynamicTest::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::DYNAMICTEST;
    }
}