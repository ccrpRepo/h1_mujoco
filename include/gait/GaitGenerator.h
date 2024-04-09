#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "gait/WaveGenerator.h"
#include "gait/FeetEndCal.h"

/*cycloid gait*/
class GaitGenerator
{
public:
    GaitGenerator(CtrlComponents *ctrlComp);
    ~GaitGenerator();
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight);
    void run(Vec32 &feetPos, Vec32 &feetVel);
    Vec3 getFootPos(int i);
    Vec3 getFootVel(int i);
    void restart();

private:
    float cycloidXYPosition(float startXY, float endXY, float phase);
    float cycloidXYVelocity(float startXY, float endXY, float phase);
    float cycloidZPosition(float startZ, float height, float phase);
    float cycloidZVelocity(float height, float phase);

    h1Robot *_robot;
    WaveGenerator *_waveG;
    Estimator *_est;
    FeetEndCal *_feetCal;
    LowlevelState *_state;
    float _gaitHeight;
    Vec2 _vxyGoal;
    float _dYawGoal;
    Vec2 *_phase, _phasePast;
    VecInt2 *_contact;
    Vec32 _startP, _endP, _idealP, _pastP;
    bool _firstRun;

};

#endif