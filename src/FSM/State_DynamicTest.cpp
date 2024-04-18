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
    // _ctrlComp->_d->qpos[0] = 0;
    // _ctrlComp->_d->qpos[1] = 0;
    // _ctrlComp->_d->qpos[2] = 1;
}

void State_DynamicTest::run()
{
    _ctrlComp->_robot->Update_Model();

    MatX Hfl,K,k;
    MatX my_H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(Hfl);
    MatX my_C = _dy->Cal_Generalize_Bias_force_Flt(true);
    K = _dy->Cal_K_Flt(k);

    // Eigen::Matrix<double,25,1> my_C = _dy->Cal_Generalize_Bias_force_Flt(true);

    Eigen::VectorXd q(_ctrlComp->_pinody->_model->nq);
    Eigen::VectorXd qd(_ctrlComp->_pinody->_model->nv);

    q(0) = _dy->_quat_xyz[4]; // x
    q(1) = _dy->_quat_xyz[5]; // y
    q(2) = 1;                 //_dy->_quat_xyz[6]; // z
    q(3) = 1;                 //_dy->_quat_xyz[1]; // qua_x
    q(4) = 0;                 //_dy->_quat_xyz[2]; // qua_y
    q(5) = 0;                 //_dy->_quat_xyz[3]; // qua_z
    q(6) = 0;                 //_dy->_quat_xyz[0]; // qua_w

    qd(0) = _dy->_robot->_v_base[3]; // vx
    qd(1) = _dy->_robot->_v_base[4]; // vy
    qd(2) = _dy->_robot->_v_base[5]; // vz
    qd(3) = _dy->_robot->_v_base[0]; // wx
    qd(4) = _dy->_robot->_v_base[1]; // wy
    qd(5) = _dy->_robot->_v_base[2]; // wz

    for(int i = 0; i < 19; i++)
    {
        q(i+7) = _dy->_q[i];
        qd(i+6) = _dy->_dq[i];
    }
    std::cout << "q: " << q.transpose() << std::endl;
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
    Eigen::VectorXd C = data->nle;
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

    int pelvis_index = model->getFrameId("pelvis");
    int rootjoint_index = model->getFrameId("root_joint");
    
    Eigen::Matrix<double, 6, 25> Jl[2];
    Eigen::Matrix<double, 6, 25> dJl[2];
    Eigen::MatrixXd my_Jl;
    Jl[0].setZero();
    Jl[1].setZero();
    my_Jl = _dy->Cal_Geometric_Jacobain(4, Coordiante::BODY);
    pinocchio::getFrameJacobian(*model, *data, joint_index[4], pinocchio::LOCAL, Jl[0]);
    pinocchio::getFrameJacobian(*model, *data, joint_index[9], pinocchio::LOCAL, Jl[1]);
    dJl[0].setZero();
    dJl[1].setZero();
    pinocchio::getFrameJacobianTimeVariation(*model, *data, joint_index[4], pinocchio::LOCAL, dJl[0]);
    pinocchio::getFrameJacobianTimeVariation(*model, *data, joint_index[9], pinocchio::LOCAL, dJl[1]);
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

    std::cout << "Q_init: " << std::endl
              << Q_init << std::endl;
    std::cout << "R_init: " << std::endl
              << R_init << std::endl;

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
    A.block(0, 0, 13, 25) = Qu.transpose() * (-H);
    A.block(0, 25, 25, 19) = Qu.transpose() * S.transpose();
    b = Qu.transpose() * C;

    MatX JF, dJF;
    JF.setZero(5 * 2, 25);
    JF.block(0, 0, 5, 25) = Jl[0].block(1, 0, 5, 25);
    JF.block(5, 0, 5, 25) = Jl[1].block(1, 0, 5, 25);

    dJF.setZero(5 * 2, 25);
    dJF.block(0, 0, 5, 25) = dJl[0].block(1, 0, 5, 25);
    dJF.block(5, 0, 5, 25) = dJl[1].block(1, 0, 5, 25);

    A.setZero(5 * 2, 44);
    b.setZero(5 * 2, 1);
    A.block(0, 0, 5 * 2, 25) = JF;
    b = -dJF * qd;

    VecX qdd;
    qdd.setOnes(25);

    Mat4 T_lfoot = data->oMi[joint_index[4]];
    Mat4 T_rfoot = data->oMi[joint_index[9]];
    Mat4 T_Base2Wrd = data->oMi[pelvis_index];
    Mat4 T_flt = data->oMi[rootjoint_index];

    Mat4 T_foot2Base = T_flt.inverse() * T_lfoot;

    std::cout
        << "T_foot2Base: " << std::endl
        << T_foot2Base << std::endl;

    std::cout
        << "T_flt: " << std::endl
        << T_flt << std::endl;

    // tau = C.tail(19);
    // _lowCmd->setTau(tau);
    _lowCmd->setQ(des_q);

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