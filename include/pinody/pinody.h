#ifndef _PINODY_H_
#define _PINODY_H_

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>

struct Pinody
{
public: 
    Pinody(pinocchio::Model* model,
           pinocchio::Data* data)
    {
        _model = model;
        _data = data;
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

        pelvis_index = model->getFrameId("pelvis");
        rootjoint_index = model->getFrameId("root_joint");
    }

    ~Pinody(){}

    pinocchio::Model* _model;
    pinocchio::Data* _data;
    Eigen::VectorXd _q;
    Eigen::VectorXd _qd;

    int joint_index[19] = {0};
    int pelvis_index;
    int rootjoint_index;

private: 
};


#endif