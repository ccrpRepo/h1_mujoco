#include "gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(CtrlComponents *ctrlComp)
    : _est(ctrlComp->estimator), _lowState(ctrlComp->lowState),
      _robot(ctrlComp->_robot)
{
    _Tstance = ctrlComp->waveGen->getTstance();
    _Tswing = ctrlComp->waveGen->getTswing();

    _kx = 0.1;
    _ky = 0.1;
    _kyaw = 0.01;

    Vec32 feetPosBody = _robot->getFeetPosIdeal();

    for (int i(0); i < 2; ++i)
    {
        _feetRadius(i) = sqrt(pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2));
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

FeetEndCal::~FeetEndCal()
{
}

Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase)
{
    _bodyVelGlobal = _est->getVelocity();
    _bodyWGlobal = _lowState->getGyroGlobal();
    Mat3 B2G_RotMat = _lowState->getRotMat();
    Mat3 G2B_RotMat = B2G_RotMat.transpose();

    _nextStep(0) = _bodyVelGlobal(0) * (1 - phase) * _Tswing + _bodyVelGlobal(0) * _Tstance / 2 + _kx * (_bodyVelGlobal(0) - vxyGoalGlobal(0));
    _nextStep(1) = _bodyVelGlobal(1) * (1 - phase) * _Tswing + _bodyVelGlobal(1) * _Tstance / 2 + _ky * (_bodyVelGlobal(1) - vxyGoalGlobal(1));

    _nextStep(2) = 0;

    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    _nextYaw = _dYaw * (1 - phase) * _Tswing + _dYaw * _Tstance / 2 + _kyaw * (dYawGoal - _dYaw);
    
    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);
    
    _footPos = _est->getPosition() + _nextStep;
    _footPos(2) = 0.0;
    
    return _footPos;
}