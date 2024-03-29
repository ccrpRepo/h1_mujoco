#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOinterface.h"
#include "interface/CmdPanel.h"
#include "rpdynamics/dynamics.h"
#include "gait/WaveGenerator.h"
#include "control/Estimator.h"

#include <string>
#include <iostream>

struct CtrlComponents
{
public:
    CtrlComponents(IOinterface *ioInter, Dynamics *dy_, mjData* d, mjModel* m) : ioInter(ioInter),dy(dy_)
    {
        // waveGen = new WaveGenerator(0.5, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot 0.45
        _d = d;
        _m = m;
        _robot = dy->_robot;
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents()
    {
        std::cout << "delete ctrl" << std::endl;
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete _robot;
        delete waveGen;
        delete estimator;
        // delete balCtrl;
        delete dy;
    }
    
    IOinterface *ioInter;
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    h1Robot *_robot;
    WaveGenerator *waveGen;
    Estimator *estimator;
    // BalanceCtrl *balCtrl;
    Dynamics *dy;
    double q[19];
    double qd[19];
    double quaxyz[7];
    double v_base[6];

    mjData *_d;
    mjModel *_m;

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void geneObj()
    {
        if (ctrlPlatform == CtrlPlatform::MUJOCO)
        {
            estimator = new Estimator(_robot, _d, _m);
        }
        else
        {
            // estimator = new Estimator(_robot, lowState, contact, phase, dt);
        }
        // balCtrl = new BalanceCtrl(robotModel);
    }
    void setAllSwing()
    {
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void sendRecv()
    {
        ioInter->sendRecv(lowCmd, lowState);
        for (int i(0); i < 19; ++i)
        {
            q[i] = lowState->motorState[i].q;
            qd[i] = lowState->motorState[i].dq;
        }
        for (int i(0); i < 3; ++i)
        {
            quaxyz[i] = lowState->imu.quaternion[i];
            v_base[i] = lowState->imu.gyroscope[i];
        }
        quaxyz[3] = lowState->imu.quaternion[3];
    }

    void runWaveGen()
    {
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void set_robot_state()
    {
        Vec3 EstPosition = estimator->getPosition();
        Vec3 EstVelocity = estimator->getVelocity();
        Mat3 B2G_RotMat = estimator->_lowState->getRotMat();
        Mat3 G2B_RotMat = B2G_RotMat.transpose();
        quaxyz[4] = EstPosition(0);
        quaxyz[5] = EstPosition(1);
        quaxyz[6] = EstPosition(2);

        // quaxyz[4] = 0;
        // quaxyz[5] = 0;
        // quaxyz[6] = 0;

        EstVelocity = G2B_RotMat * EstVelocity;
        v_base[3] = EstVelocity(0);
        v_base[4] = EstVelocity(1);
        v_base[5] = EstVelocity(2);

        dy->_robot->set_q(q);
        dy->_robot->set_dq(qd);
        dy->_robot->set_quaxyz(quaxyz);
        dy->_robot->set_vbase(v_base);
    }
    

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
};



#endif // CTRLCOMPONENTS_H