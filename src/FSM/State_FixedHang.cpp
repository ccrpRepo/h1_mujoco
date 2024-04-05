#include "FSM/State_FixedHang.h"

State_FixedHang::State_FixedHang(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixedhang") {}

void State_FixedHang::enter()
{
    if (_ctrlComp->ctrlPlatform == CtrlPlatform::MUJOCO)
    {
        for (int i = 0; i < 19; i++)
        {
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 100;
            _lowCmd->motorCmd[i].Kd = 10;
            _lowCmd->motorCmd[i].tau = 0;
        }
        _lowCmd->motorCmd[2].mode = 10;
        _lowCmd->motorCmd[2].q = -0.4;
        _lowCmd->motorCmd[2].dq = 0;
        _lowCmd->motorCmd[2].Kp = 100;
        _lowCmd->motorCmd[2].Kd = 10;
        _lowCmd->motorCmd[2].tau = 0;

        _lowCmd->motorCmd[3].mode = 10;
        _lowCmd->motorCmd[3].q = 0.8;
        _lowCmd->motorCmd[3].dq = 0;
        _lowCmd->motorCmd[3].Kp = 100;
        _lowCmd->motorCmd[3].Kd = 10;
        _lowCmd->motorCmd[3].tau = 0;

        _lowCmd->motorCmd[4].mode = 10;
        _lowCmd->motorCmd[4].q = -0.4;
        _lowCmd->motorCmd[4].dq = 0;
        _lowCmd->motorCmd[4].Kp = 100;
        _lowCmd->motorCmd[4].Kd = 10; 
        _lowCmd->motorCmd[4].tau = 0;

        _lowCmd->motorCmd[7].mode = 10;
        _lowCmd->motorCmd[7].q = -0.4;
        _lowCmd->motorCmd[7].dq = 0;
        _lowCmd->motorCmd[7].Kp = 100;
        _lowCmd->motorCmd[7].Kd = 10;
        _lowCmd->motorCmd[7].tau = 0;

        _lowCmd->motorCmd[8].mode = 10;
        _lowCmd->motorCmd[8].q = 0.8;
        _lowCmd->motorCmd[8].dq = 0;
        _lowCmd->motorCmd[8].Kp = 100;
        _lowCmd->motorCmd[8].Kd = 10;
        _lowCmd->motorCmd[8].tau = 0;

        _lowCmd->motorCmd[9].mode = 10;
        _lowCmd->motorCmd[9].q = -0.4;
        _lowCmd->motorCmd[9].dq = 0;
        _lowCmd->motorCmd[9].Kp = 100;
        _lowCmd->motorCmd[9].Kd = 10;
        _lowCmd->motorCmd[9].tau = 0;
    }
    else if (_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)
    {
        for (int i = 0; i < 19; i++)
        {
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 3;
            _lowCmd->motorCmd[i].tau = 0;
        }
    }

    _ctrlComp->setAllSwing();
}

void State_FixedHang::run()
{
    // std::cout << "passive mode" << std::endl;
    
}

void State_FixedHang::exit()
{
}

FSMStateName State_FixedHang::checkChange()
{
    if (_lowState->userCmd == UserCommand::START)
    {
        return FSMStateName::STEPTEST;
        // return FSMStateName::SWINGTEST;
    }
    else if (_lowState->userCmd == UserCommand::L1_X)
    {
        return FSMStateName::DYNAMICTEST;
    }
    else if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::FIXEDSTAND;
    }
}