#include "FSM/State_StepWalking.h"

State_StepWalking::State_StepWalking(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::STEPWALKING, "step walking"),
      _est(ctrlComp->estimator), _robot(ctrlComp->_robot),
      _contact(ctrlComp->contact),
      _phase(ctrlComp->phase)
{
    _wbc = new WBC(_ctrlComp->dy);
}

void State_StepWalking::enter()
{

}

void State_StepWalking::run()
{
}

void State_StepWalking::exit()
{
}


FSMStateName State_StepWalking::checkChange()
{
}