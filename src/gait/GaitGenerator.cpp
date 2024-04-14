#include "gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
    : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator),
      _phase(ctrlComp->phase), _contact(ctrlComp->contact),
      _robot(ctrlComp->_robot), _state(ctrlComp->lowState)
{

    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

GaitGenerator::~GaitGenerator()
{
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight)
{
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart()
{
    _firstRun = true;
    _vxyGoal.setZero();
}

void GaitGenerator::run(Vec32 &feetPos, Vec32 &feetVel)
{
    static int timer = 0;
    if (_firstRun)
    {
        _startP = _est->getFeetPos();
        _firstRun = false;
    }
    
    for (int i(0); i < 2; ++i)
    {
        if ((*_contact)(i) == 1)
        {
            if ((*_phase)(i) < 0.5)
            {
                _startP.col(i) = _est->getFootPos(i);
            }
            // _startP.col(0) << 0, 0.2, 0;
            // _startP.col(1) << 0, -0.2, 0;
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else
        {
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));
            // _endP.col(0) << 0, 0.2, 0;
            // _endP.col(1) << 0, -0.2, 0;
            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
        
    }
    Mat4 T_cur;
    T_cur.setIdentity(4, 4);

    for (int i = 0; i < 5; i++)
    {
        T_cur = T_cur * _robot->T_dwtree[i];
    }
    timer++;
    // if(timer>50)
    // {
    //     timer = 0;
    //     std::cout
    //         << "_phase: " << (*_phase)(0);
    //     if ((*_contact)(0) == 1)
    //         std::cout << "  left contact";
    //     std::cout << std::endl;
    //     std::cout << "cur_foot1: " << _est->getFootPos(0).transpose() << std::endl;
    //     std::cout << "cur_foot2: " << T_cur.block(0,3,3,1).transpose() << std::endl;
    //     std::cout
    //         << "_startP: " << _startP.col(0).transpose() << std::endl;
    //     std::cout << "desPos: " << feetPos.col(0).transpose() << std::endl;
    //     std::cout << "_endP: " << _endP.col(0).transpose() << std::endl
    //               << std::endl;
    // }
    
    _pastP = feetPos;
    _phasePast = *_phase;
}

Vec3 GaitGenerator::getFootPos(int i)
{
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) = cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));

    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i)
{
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) = cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

float GaitGenerator::cycloidXYPosition(float start, float end, float phase)
{
    float phasePI = 2 * M_PI * phase;
    return (end - start) * (phasePI - sin(phasePI)) / (2 * M_PI) + start;
}

float GaitGenerator::cycloidXYVelocity(float start, float end, float phase)
{
    float phasePI = 2 * M_PI * phase;
    return (end - start) * (1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase)
{
    float phasePI = 2 * M_PI * phase;
    return h * (1 - cos(phasePI)) / 2 + start;
}

float GaitGenerator::cycloidZVelocity(float h, float phase)
{
    float phasePI = 2 * M_PI * phase;
    return h * M_PI * sin(phasePI) / _waveG->getTswing();
}