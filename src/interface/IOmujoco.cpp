#include "interface/IOmujoco.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include "rpdynamics/TimeCounter.h"

IOmujoco::IOmujoco(mjData *d, mjModel *m) : IOinterface()
{
    _d = d;
    _m = m;
    cmdPanel = new KeyBoard();
}

IOmujoco::~IOmujoco()
{
    delete cmdPanel;
}

void IOmujoco::sendCmd(const LowlevelCmd *lowCmd)
{
    for (int i(0); i < 19; ++i)
    {
        _lowCmd.motorcmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motorcmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motorcmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motorcmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motorcmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorcmd[i].Kp = lowCmd->motorCmd[i].Kp;
    }
    for (int j(0); j < 19; ++j)
    {
        _d->ctrl[j] = _lowCmd.motorcmd[j].tau
                    + _lowCmd.motorcmd[j].Kp * (_lowCmd.motorcmd[j].q - _lowState.motorstate[j].q)
                    + _lowCmd.motorcmd[j].Kd * (_lowCmd.motorcmd[j].dq - _lowState.motorstate[j].dq);
        // std::cout << _d->ctrl[j] << ", ";
    }
    // std::cout << std::endl;
}

void IOmujoco::recvState(LowlevelState *state)
{
    for (int i(0); i < 19; ++i)
    {
        state->motorState[i].q = _d->sensordata[0 + i];
        _lowState.motorstate[i].q = _d->sensordata[0 + i];
        state->motorState[i].dq = _d->sensordata[19 + i];
        _lowState.motorstate[i].dq = _d->sensordata[19 + i];
        state->motorState[i].ddq = _d->qacc[i + 6];
        state->motorState[i].tauEst = _d->actuator_force[i];
        _lowState.motorstate[i].tauEst = _d->actuator_force[i];
        // std::cout << _lowState.motorstate[i].dq << ", ";
    }
    for (int i(0); i < 3; ++i)
    {
        _lowState.imustate.quaternion[i] = _d->sensordata[47 + i];
        state->imu.quaternion[i] = _d->sensordata[47 + i];
        _lowState.imustate.accelermeter[i] = _d->sensordata[38 + i];
        state->imu.accelerometer[i] = _d->sensordata[38 + i];
        _lowState.imustate.gyroscope[i] = _d->sensordata[41 + i];
        state->imu.gyroscope[i] = _d->sensordata[41 + i];
        // std::cout << _lowState.imustate.quaternion[i] << ", ";
    }
    _lowState.imustate.quaternion[3] = _d->sensordata[50];
    state->imu.quaternion[3] = _d->sensordata[50];
    // std::cout <<_lowState.imustate.quaternion[3] << ", ";
    // std::cout << std::endl;
}

void IOmujoco::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);
    recvState(state);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    
}