#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    : _ctrlComp(ctrlComp)
{
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
    _stateList.fixedHang = new State_FixedHang(_ctrlComp);
    _stateList.dynamicTest = new State_DynamicTest(_ctrlComp);
    _stateList.balanceStand = new State_BalanceStand(_ctrlComp);
    _stateList.stepwalking = new State_StepWalking(_ctrlComp);
    _stateList.invalid = nullptr;

    initialize();
}

FSM::~FSM()
{
    _stateList.deletePtr();
}

void FSM::initialize()
{
    _currentState = _stateList.passive;
    _currentState->enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    _startTime = getSystemTime();
    
    _ctrlComp->sendRecv();
    _ctrlComp->_robot->Update_Model();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run_inMujoco();
    
    _ctrlComp->set_robot_state();

    if (!checkSafty())
    {
        _ctrlComp->ioInter->setPassive();
    }
    if (_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if (_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);

            std::cout
                << "Switched from " << _currentState->_stateNameString
                << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if (_mode == FSMMode::CHANGE)
    {
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();

        _mode = FSMMode::NORMAL;
        _currentState->run();
    }
    // for (int i = 0; i < 12;i++)
    // {
    //     std::cout << _ctrlComp->lowCmd->motorCmd[i].q << ", ";
    // }
    // std::cout << std::endl;

    _ctrlComp->dy->_robot->_isUpdated = false;
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState *FSM::getNextState(FSMStateName stateName)
{
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDHANG:
        return _stateList.fixedHang;
        break;
    case FSMStateName::DYNAMICTEST:
        return _stateList.dynamicTest;
        break;
    case FSMStateName::BALANCESTAND:
        return _stateList.balanceStand;
        break;
    case FSMStateName::STEPWALKING:
        return _stateList.stepwalking;
        break;

    // case FSMStateName::FIXEDSTAND:
    //     return _stateList.fixedStand;
    //     break;
    // case FSMStateName::FREESTAND:
    //     return _stateList.freeStand;
    //     break;
    // case FSMStateName::TROTTING:
    //     return _stateList.trotting;
    //     break;
    // case FSMStateName::BALANCETEST:
    //     return _stateList.balanceTest;
    //     break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty()
{
    // The angle with z axis less than 60 degree
    if (_ctrlComp->lowState->getRotMat()(2, 2) < 0.5)
    {
        return false;
    }
    else
    {
        return true;
    }
}