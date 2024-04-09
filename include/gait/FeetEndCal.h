#ifndef FEETENDCAL_H
#define FEETENDCAL_H

#include "control/CtrlComponents.h"
#include "message/LowlevelState.h"

class FeetEndCal
{
public:
    FeetEndCal(CtrlComponents *ctrlComp);
    ~FeetEndCal();
    Vec3 calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase);

private:
    LowlevelState *_lowState;
    Estimator *_est;
    h1Robot *_robot;

    Vec3 _nextStep, _footPos;
    Vec3 _bodyVelGlobal; // linear velocity
    Vec3 _bodyAccGlobal; // linear accelerator
    Vec3 _bodyWGlobal;   // angular velocity

    Vec2 _feetRadius, _feetInitAngle;
    float _yaw, _dYaw, _nextYaw;

    float _Tstance, _Tswing;
    float _kx, _ky, _kyaw;
};

#endif