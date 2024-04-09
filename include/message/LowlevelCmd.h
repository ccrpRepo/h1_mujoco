#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"

struct MotorCmd
{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd()
    {
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowlevelCmd
{
    MotorCmd motorCmd[19];

    void setQ(Eigen::Matrix<double,19,1> q)
    {
        for (int i(0); i < 19; ++i)
        {
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec5 qi)
    {
        motorCmd[legID * 5 + 0].q = qi(0);
        motorCmd[legID * 5 + 1].q = qi(1);
        motorCmd[legID * 5 + 2].q = qi(2);
        motorCmd[legID * 5 + 3].q = qi(3);
        motorCmd[legID * 5 + 4].q = qi(4);
    }
    void setQd(Eigen::Matrix<double, 19, 1> qd)
    {
        for (int i(0); i < 19; ++i)
        {
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3 qdi)
    {
        motorCmd[legID * 5 + 0].dq = qdi(0);
        motorCmd[legID * 5 + 1].dq = qdi(1);
        motorCmd[legID * 5 + 2].dq = qdi(2);
        motorCmd[legID * 5 + 3].dq = qdi(3);
        motorCmd[legID * 5 + 4].dq = qdi(4);
    }
    void setTau(Eigen::Matrix<double, 19, 1> tau, Vec2 torqueLimit = Vec2(-50, 50))
    {
        for (int i(0); i < 19; ++i)
        {
            if (std::isnan(tau(i)))
            {
                // printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }
    void setZeroDq(int legID)
    {
        motorCmd[legID * 5 + 0].dq = 0;
        motorCmd[legID * 5 + 1].dq = 0;
        motorCmd[legID * 5 + 2].dq = 0;
        motorCmd[legID * 5 + 3].dq = 0;
        motorCmd[legID * 5 + 4].dq = 0;
    }
    void setZeroDq()
    {
        for (int i(0); i < 2; ++i)
        {
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID)
    {
        motorCmd[legID * 5 + 0].tau = 0;
        motorCmd[legID * 5 + 1].tau = 0;
        motorCmd[legID * 5 + 2].tau = 0;
        motorCmd[legID * 5 + 3].tau = 0;
        motorCmd[legID * 5 + 4].tau = 0;
    }
    void setSimStanceGain(int legID)
    {
        motorCmd[legID * 5 + 0].mode = 10;
        motorCmd[legID * 5 + 0].Kp = 100; //180
        motorCmd[legID * 5 + 0].Kd = 20; //8
        motorCmd[legID * 5 + 1].mode = 10;
        motorCmd[legID * 5 + 1].Kp = 100; //180
        motorCmd[legID * 5 + 1].Kd = 20; //8
        motorCmd[legID * 5 + 2].mode = 10;
        motorCmd[legID * 5 + 2].Kp = 200; //300
        motorCmd[legID * 5 + 2].Kd = 20; //15
        motorCmd[legID * 5 + 3].mode = 10;
        motorCmd[legID * 5 + 3].Kp = 200; // 180
        motorCmd[legID * 5 + 3].Kd = 20;   // 8
        motorCmd[legID * 5 + 4].mode = 10;
        motorCmd[legID * 5 + 4].Kp = 100; // 300
        motorCmd[legID * 5 + 4].Kd = 15; // 15
    }
    void setRealStanceGain(int legID)
    {
        motorCmd[legID * 5 + 0].mode = 10;
        motorCmd[legID * 5 + 0].Kp = 60;
        motorCmd[legID * 5 + 0].Kd = 5;
        motorCmd[legID * 5 + 1].mode = 10;
        motorCmd[legID * 5 + 1].Kp = 40;
        motorCmd[legID * 5 + 1].Kd = 4;
        motorCmd[legID * 5 + 2].mode = 10;
        motorCmd[legID * 5 + 2].Kp = 80;
        motorCmd[legID * 5 + 2].Kd = 7;
        motorCmd[legID * 5 + 3].mode = 10;
        motorCmd[legID * 5 + 3].Kp = 40;
        motorCmd[legID * 5 + 3].Kd = 4;
        motorCmd[legID * 5 + 4].mode = 10;
        motorCmd[legID * 5 + 4].Kp = 80;
        motorCmd[legID * 5 + 4].Kd = 7;
    }
    void setZeroGain(int legID)
    {
        motorCmd[legID * 5 + 0].mode = 10;
        motorCmd[legID * 5 + 0].Kp = 0;
        motorCmd[legID * 5 + 0].Kd = 0;
        motorCmd[legID * 5 + 1].mode = 10;
        motorCmd[legID * 5 + 1].Kp = 0;
        motorCmd[legID * 5 + 1].Kd = 0;
        motorCmd[legID * 5 + 2].mode = 10;
        motorCmd[legID * 5 + 2].Kp = 0;
        motorCmd[legID * 5 + 2].Kd = 0;
        motorCmd[legID * 5 + 3].mode = 10;
        motorCmd[legID * 5 + 3].Kp = 0;
        motorCmd[legID * 5 + 3].Kd = 0;
        motorCmd[legID * 5 + 4].mode = 10;
        motorCmd[legID * 5 + 4].Kp = 0;
        motorCmd[legID * 5 + 4].Kd = 0;
    }

    void setArmGain()
    {
        for (int i = 11; i < 19; i++)
        {
            motorCmd[i].mode = 10;
            motorCmd[i].Kp = 20;
            motorCmd[i].Kd = 2;
        }
    }

    void setTorsoGain()
    {
        motorCmd[10].mode = 10;
        motorCmd[10].Kp = 20;
        motorCmd[10].Kd = 2;
        
    }

    void setWholeZeroGain()
    {
        for (int i = 0; i < 19;i++)
        {
            motorCmd[i].mode = 10;
            motorCmd[i].Kp = 0;
            motorCmd[i].Kd = 0;
        }
            
    }
    void setWholeSmallGain()
    {
        for (int i = 0; i < 19; i++)
        {
            motorCmd[i].mode = 10;
            motorCmd[i].Kp = 0.3;
            motorCmd[i].Kd = 0;
        }
    }
    void setZeroGain()
    {
        for (int i(0); i < 2; ++i)
        {
            setZeroGain(i);
        }
    }
    void setStableGain(int legID)
    {
        motorCmd[legID * 5 + 0].mode = 10;
        motorCmd[legID * 5 + 0].Kp = 10;
        motorCmd[legID * 5 + 0].Kd = 2;
        motorCmd[legID * 5 + 1].mode = 10;
        motorCmd[legID * 5 + 1].Kp = 10;
        motorCmd[legID * 5 + 1].Kd = 2;
        motorCmd[legID * 5 + 2].mode = 10;
        motorCmd[legID * 5 + 2].Kp = 10;
        motorCmd[legID * 5 + 2].Kd = 2;
        motorCmd[legID * 5 + 3].mode = 10;
        motorCmd[legID * 5 + 3].Kp = 10;
        motorCmd[legID * 5 + 3].Kd = 2;
        motorCmd[legID * 5 + 4].mode = 10;
        motorCmd[legID * 5 + 4].Kp = 10;
        motorCmd[legID * 5 + 4].Kd = 2;
    }
    void setStableGain()
    {
        for (int i(0); i < 2; ++i)
        {
            setStableGain(i);
        }
    }
    void setSwingGain(int legID)
    {
        motorCmd[legID * 5 + 0].mode = 10;
        motorCmd[legID * 5 + 0].Kp = 200;
        motorCmd[legID * 5 + 0].Kd = 50;
        motorCmd[legID * 5 + 1].mode = 10;
        motorCmd[legID * 5 + 1].Kp = 200;
        motorCmd[legID * 5 + 1].Kd = 50;
        motorCmd[legID * 5 + 2].mode = 10;
        motorCmd[legID * 5 + 2].Kp = 200;
        motorCmd[legID * 5 + 2].Kd = 50;
        motorCmd[legID * 5 + 3].mode = 10;
        motorCmd[legID * 5 + 3].Kp = 200;
        motorCmd[legID * 5 + 3].Kd = 50;
        motorCmd[legID * 5 + 4].mode = 10;
        motorCmd[legID * 5 + 4].Kp = 200;
        motorCmd[legID * 5 + 4].Kd = 50;
    }
};

#endif