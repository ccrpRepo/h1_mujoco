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
        _lowCmd->setWholeSmallGain();
    }
    else if (_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)
    {
        // _lowCmd->setRealStanceGain(i);
    }
}

void State_DynamicTest::run()
{
    _ctrlComp->_robot->Update_Model();

    Eigen::Matrix<double, 19, 1> tau;
    tau = _dy->Cal_Generalize_Bias_force(true);

    _lowCmd->setTau(tau);
    Eigen::Matrix<double, 19, 1> des_q;
    des_q << 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0,
             2,
             0, 0, 0, 0,
             0, 0, 0, 0;
    _lowCmd->setQ(des_q);
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