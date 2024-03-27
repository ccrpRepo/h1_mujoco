#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOinterface.h"
#include "interface/CmdPanel.h"
#include "common/robot.h"
#include "rpdynamics/dynamics.h"

#include <string>
#include <iostream>

struct CtrlComponents
{
public:
    CtrlComponents(IOinterface *ioInter, Dynamics *dy_) : ioInter(ioInter),dy(dy_)
    {
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt2;
        phase = new Vec2;
        *contact = VecInt2(0, 0);
        *phase = Vec2(0.5, 0.5);
    }

    IOinterface *ioInter;
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    QuadrupedRobot *robotModel;
    // WaveGenerator *waveGen;
    // Estimator *estimator;
    // BalanceCtrl *balCtrl;
    Dynamics *dy;
    double q[12];
    double qd[12];
    double quaxyz[7];
    double v_base[6];

    VecInt2 *contact;
    Vec2 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

private:

};



#endif // CTRLCOMPONENTS_H