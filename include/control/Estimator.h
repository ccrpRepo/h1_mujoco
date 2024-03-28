#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <vector>
#include "rpdynamics/RPRobot.h"
#include "common/LowPassFilter.h"
#include "gait/WaveGenerator.h"
#include "message/LowlevelState.h"
#include "string"
#include "interface/IOmujoco.h"

class Estimator
{
public:
    // Estimator(h1Robot *robot, LowlevelState *lowState, VecInt2 *contact, Vec2 *phase, double dt);
    // Estimator(h1Robot *robot, LowlevelState *lowState, VecInt2 *contact, Vec2 *phase, double dt, Vec18 Qdig, std::string testName);
    Estimator(h1Robot *robot, mjData *d, mjModel *m);
    ~Estimator();
    Vec3 getPosition();
    Vec3 getVelocity();
    Vec3 getFootPos(int i);
    Eigen::Matrix<double, 3, 2> getFeetPos();
    Eigen::Matrix<double, 3, 2> getFeetVel();
    Eigen::Matrix<double, 3, 2> getPosFeet2BGlobal();
    void run();
    void run_inMujoco();
    LowlevelState *_lowState;

private:
    void _initSystem();
    // Linear System
    Eigen::Matrix<double, 12, 1> _xhat; // The state of estimator, position(3)+velocity(3)+feet position(3x2)
    // Vec3 _u;                            // The input of estimator
    // Eigen::Matrix<double, 28, 1> _y;    // The measurement value of output y
    // Eigen::Matrix<double, 28, 1> _yhat; // The prediction of output y
    // Eigen::Matrix<double, 18, 18> _A;   // The transtion matrix of estimator
    // Eigen::Matrix<double, 18, 3> _B;    // The input matrix
    // Eigen::Matrix<double, 28, 18> _C;   // The output matrix
    // // Covariance Matrix
    // Eigen::Matrix<double, 18, 18> _P;       // Prediction covariance
    // Eigen::Matrix<double, 18, 18> _Ppriori; // Priori prediction covariance
    // Eigen::Matrix<double, 18, 18> _Q;       // Dynamic simulation covariance
    // Eigen::Matrix<double, 28, 28> _R;       // Measurement covariance
    // Eigen::Matrix<double, 18, 18> _QInit;   // Initial value of Dynamic simulation covariance
    // Eigen::Matrix<double, 28, 28> _RInit;   // Initial value of Measurement covariance
    Vec18 _Qdig;                            // adjustable process noise covariance
    // Mat3 _Cu;                               // The covariance of system input u
    // // Output Measurement
    // Eigen::Matrix<double, 12, 1> _feetPos2Body;              // The feet positions to body, in the global coordinate
    // Eigen::Matrix<double, 12, 1> _feetVel2Body;              // The feet velocity to body, in the global coordinate
    // Eigen::Matrix<double, 4, 1> _feetH;                      // The Height of each foot, in the global coordinate
    // Eigen::Matrix<double, 28, 28> _S;                        // _S = C*P*C.T + R
    // Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> _Slu; // _S.lu()
    // Eigen::Matrix<double, 28, 1> _Sy;                        // _Sy = _S.inv() * (y - yhat)
    // Eigen::Matrix<double, 28, 18> _Sc;                       // _Sc = _S.inv() * C
    // Eigen::Matrix<double, 28, 28> _SR;                       // _SR = _S.inv() * R
    // Eigen::Matrix<double, 28, 18> _STC;                      // _STC = (_S.transpose()).inv() * C
    // Eigen::Matrix<double, 18, 18> _IKC;                      // _IKC = I - KC

    RotMat _rotMatB2G; // Rotate Matrix: from body to global
    Vec3 _g;
    Vec34 _feetPosGlobalKine, _feetVelGlobalKine;

    h1Robot *_robot;
    Vec2 *_phase;
    VecInt2 *_contact;
    double _dt;
    double _trust;
    double _largeVariance;

    mjData *_d;
    mjModel *_m;

    // Low pass filters
    LPFilter *_vxFilter,
        *_vyFilter, *_vzFilter;

    // Tuning
    AvgCov *_RCheck;
    AvgCov *_uCheck;
    std::string _estName;
};

#endif