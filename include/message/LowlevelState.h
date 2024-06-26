#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

struct MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4]; // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];

    IMU()
    {
        for (int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
        quaternion[0] = 1;
    }

    RotMat getRotMat()
    {
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc()
    {
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro()
    {
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat()
    {
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu;
    MotorState motorState[19];
    UserCommand userCmd;
    UserValue userValue;

    Mat52 getQ()
    {
        Mat52 qLegs;
        for (int i(0); i < 2; ++i)
        {
            qLegs.col(i)(0) = motorState[5 * i].q;
            qLegs.col(i)(1) = motorState[5 * i + 1].q;
            qLegs.col(i)(2) = motorState[5 * i + 2].q;
            qLegs.col(i)(3) = motorState[5 * i + 3].q;
            qLegs.col(i)(4) = motorState[5 * i + 4].q;
        }
        return qLegs;
    }

    Mat52 getQd()
    {
        Mat52 qdLegs;
        for (int i(0); i < 2; ++i)
        {
            qdLegs.col(i)(0) = motorState[5 * i].dq;
            qdLegs.col(i)(1) = motorState[5 * i + 1].dq;
            qdLegs.col(i)(2) = motorState[5 * i + 2].dq;
            qdLegs.col(i)(3) = motorState[5 * i + 3].dq;
            qdLegs.col(i)(4) = motorState[5 * i + 4].dq;
        }
        return qdLegs;
    }

    RotMat getRotMat()
    {
        return imu.getRotMat();
    }

    Vec3 getAcc()
    {
        return imu.getAcc();
    }

    Vec3 getGyro()
    {
        return imu.getGyro();
    }

    Vec3 getAccGlobal()
    {
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal()
    {
        return getRotMat() * getGyro();
    }

    double getYaw()
    {
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw()
    {
        return getGyroGlobal()(2);
    }

    void setQ(Eigen::Matrix<double,19,1> q)
    {
        for (int i(0); i < 19; ++i)
        {
            motorState[i].q = q(i);
        }
    }
};

#endif // LOWLEVELSTATE_HPP