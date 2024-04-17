#include <pinocchio/fwd.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "FSM/State_DynamicTest.h"

State_DynamicTest::State_DynamicTest(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::DYNAMICTEST, "dynamicTest")
{
    _dy = _ctrlComp->dy;
}

void State_DynamicTest::enter()
{
    if (_ctrlComp->ctrlPlatform == CtrlPlatform::MUJOCO)
    {
        _lowCmd->setWholeSmallGain();
        // _lowCmd->setWholeZeroGain();
    }
    else if (_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)
    {
        // _lowCmd->setRealStanceGain(i);
    }

    des_q << 0, 0, -0.2, 0.4, 0.3,
        0, 0, -0.2, 0.4, 0.3,
        0,
        1, 1, 0, 0,
        1, -1, 0, 0;

    for (int i = 0; i < 19; i++)
    {
        _ctrlComp->_d->qpos[i] = des_q(i);
    }
}

void State_DynamicTest::run()
{
    _ctrlComp->_robot->Update_Model();

    Eigen::Matrix<double, 19, 1> tau;
    MatX Hfl,K,k;
    MatX my_H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(Hfl);
    K = _dy->Cal_K_Flt(k);


    // Eigen::Matrix<double,25,1> my_C = _dy->Cal_Generalize_Bias_force_Flt(true);

    Eigen::VectorXd q(_ctrlComp->_pinody->_model->nq);
    Eigen::VectorXd qd(_ctrlComp->_pinody->_model->nv);

    q.setZero();
    q.segment(3, 4) << 0, 0, 0, 1; // x y z w
    qd.setZero();

    for(int i = 0; i < 19; i++)
    {
        q(i+7) = _dy->_q[i];
        qd(i+6) = _dy->_dq[i];
    }

    pinocchio::Model *model;
    pinocchio::Data *data;
    model = _ctrlComp->_pinody->_model;
    data = _ctrlComp->_pinody->_data;
    
    pinocchio::forwardKinematics(*(model),*(data),q,qd);
    pinocchio::framesForwardKinematics(*model, *data, q);
    pinocchio::computeJointJacobians(*(model), *(data));
    pinocchio::updateFramePlacements(*model, *data);
    pinocchio::nonLinearEffects(*(model),*(data),q,qd);
    pinocchio::computeJointJacobiansTimeVariation(*model, *data, q, qd);

    // std::cout<<q.transpose()<<std::endl;
    // std::cout<<qd.transpose()<<std::endl;
    Eigen::VectorXd C_ = data->nle;
    Eigen::MatrixXd H = data->M;
    int joint_index[19] = {0};
    joint_index[0] = model->getFrameId("left_hip_yaw_joint");
    joint_index[1] = model->getFrameId("left_hip_roll_joint");
    joint_index[2] = model->getFrameId("left_hip_pitch_joint");
    joint_index[3] = model->getFrameId("left_knee_joint");
    joint_index[4] = model->getFrameId("left_ankle_joint");
    joint_index[5] = model->getFrameId("right_hip_yaw_joint");
    joint_index[6] = model->getFrameId("right_hip_roll_joint");
    joint_index[7] = model->getFrameId("right_hip_pitch_joint");
    joint_index[8] = model->getFrameId("right_knee_joint");
    joint_index[9] = model->getFrameId("right_ankle_joint");
    joint_index[10] = model->getFrameId("torso_link");
    joint_index[11] = model->getFrameId("left_shoulder_pitch_joint");
    joint_index[12] = model->getFrameId("left_shoulder_roll_joint");
    joint_index[13] = model->getFrameId("left_shoulder_yaw_joint");
    joint_index[14] = model->getFrameId("left_elbow_joint");
    joint_index[15] = model->getFrameId("right_shoulder_pitch_joint");
    joint_index[16] = model->getFrameId("right_shoulder_roll_joint");
    joint_index[17] = model->getFrameId("right_shoulder_yaw_joint");
    joint_index[18] = model->getFrameId("right_elbow_joint");

    Eigen::Matrix<double, 6, 25> Jl[2];
    Eigen::Matrix<double, 6, 25> dJl;
    Eigen::MatrixXd my_Jl;
    Jl[0].setZero();
    Jl[1].setZero();
    my_Jl = _dy->Cal_Geometric_Jacobain(4, Coordiante::BODY);
    pinocchio::getFrameJacobian(*model, *data, joint_index[4], pinocchio::LOCAL_WORLD_ALIGNED, Jl[0]);
    pinocchio::getFrameJacobian(*model, *data, joint_index[9], pinocchio::LOCAL_WORLD_ALIGNED, Jl[1]);
    dJl.setZero();
    pinocchio::getFrameJacobianTimeVariation(*model, *data, joint_index[4], pinocchio::LOCAL_WORLD_ALIGNED, dJl);

    // std::cout <<"avp: " <<_dy->_avp[4].transpose() << std::endl;

    // std::cout << "djldq: " << qd.transpose() * dJl.transpose() << std::endl;
    MatX S;
    S.setZero(19, 25);
    S.block(0, 6, 19, 19).setIdentity(19, 19);

    MatX Js;
    Js.setZero(12, 25);
    Js.block(0, 0, 6, 25) = Jl[0];
    Js.block(6, 0, 6, 25) = Jl[1];

    MatX Js_T = Js.transpose();

    Eigen::HouseholderQR<Eigen::MatrixXd> qr = Js_T.householderQr();
    Eigen::MatrixXd Q_init = qr.householderQ();
    Eigen::MatrixXd R_init = qr.matrixQR().triangularView<Eigen::Upper>();

    MatX Qc, Qu, R;
    Qc.setZero(25, 12);
    Qu.setZero(25, 13);
    R.setZero(12, 12);
    Qc = Q_init.block(0, 0, 25, 12);
    Qu = Q_init.block(0, 12, 25, 13);
    R = R_init.block(0, 0, 12, 12);

    MatX A, b;
    A.setZero(25, 44);
    b.setZero(13, 1);
    A.block(0, 0, 13, 25) = Qu.transpose() * (-H.transpose());
    A.block(0, 25, 25, 19) = Qu.transpose() * S.transpose();
    b = Qu.transpose() * C_;

    // tau = C_.tail(19);
    // _lowCmd->setTau(tau);
    // _lowCmd->setQ(des_q);

    // _ctrlComp->_d->qfrc_applied[1] = 1;
}

void State_DynamicTest::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_DynamicTest::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::DYNAMICTEST;
    }
}