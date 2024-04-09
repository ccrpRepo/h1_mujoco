#include "FSM/State_StepWalking.h"

State_StepWalking::State_StepWalking(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::STEPWALKING, "step walking"),
      _est(ctrlComp->estimator), _robot(ctrlComp->_robot),
      _contact(ctrlComp->contact),
      _phase(ctrlComp->phase)
{
    _wbc = _ctrlComp->_wbc;
}

void State_StepWalking::enter()
{

}

void State_StepWalking::run()
{
    _ctrlComp->_robot->Update_Model();
    _wbc->set_contact_frition(0.35);

    

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