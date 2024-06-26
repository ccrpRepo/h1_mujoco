#include "FSM/State_Passive.h"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::PASSIVE, "passive") {}

void State_Passive::enter()
{
    // std::cout << "enter passive mode" << std::endl;
    if (_ctrlComp->ctrlPlatform == CtrlPlatform::MUJOCO)
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

void State_Passive::run()
{
    // std::cout << "passive mode" << std::endl;
}

void State_Passive::exit()
{
}

FSMStateName State_Passive::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::BALANCESTAND;
        // return FSMStateName::FIXEDHANG;
        // return FSMStateName::SWINGTEST;
        // return FSMStateName::DYNAMICTEST;
        // return FSMStateName::STEPWALKING;
    }
    else
    {
        return FSMStateName::PASSIVE;
    }
}