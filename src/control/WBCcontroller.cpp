#include "control/WBCController.h"

void WBC::dynamics_consistence_task(VecInt2 contact)
{
    int contact_num = 0;
    MatX Js;
    // MatX K_temp, k_temp;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    Js.setZero(contact_num * 5, 25);
    _pinody->_q.setZero(_pinody->_model->nq);
    _pinody->_qd.setZero(_pinody->_model->nv);
    _pinody->_q(0) = _dy->_quat_xyz[4]; // x
    _pinody->_q(1) = _dy->_quat_xyz[5]; // y
    _pinody->_q(2) = _dy->_quat_xyz[6]; // z
    _pinody->_q(3) = _dy->_quat_xyz[1]; // qua_x
    _pinody->_q(4) = _dy->_quat_xyz[2]; // qua_y
    _pinody->_q(5) = _dy->_quat_xyz[3]; // qua_z
    _pinody->_q(6) = _dy->_quat_xyz[0]; // qua_w

    _pinody->_qd(0) = _dy->_robot->_v_base[3]; // vx
    _pinody->_qd(1) = _dy->_robot->_v_base[4]; // vy
    _pinody->_qd(2) = _dy->_robot->_v_base[5]; // vz
    _pinody->_qd(3) = _dy->_robot->_v_base[0]; // wx
    _pinody->_qd(4) = _dy->_robot->_v_base[1]; // wy
    _pinody->_qd(5) = _dy->_robot->_v_base[2]; // wz
    for (int i = 0; i < 19; i++)
    {
        _pinody->_q(i + 7) = _dy->_q[i];
        _pinody->_qd(i + 6) = _dy->_dq[i];
    }

    pinocchio::Model *model;
    pinocchio::Data *data;
    model = _pinody->_model;
    data = _pinody->_data;

    pinocchio::forwardKinematics(*(model), *(data), _pinody->_q, _pinody->_qd);
    pinocchio::framesForwardKinematics(*model, *data, _pinody->_q);
    pinocchio::computeJointJacobians(*(model), *(data));
    pinocchio::updateFramePlacements(*model, *data);
    pinocchio::nonLinearEffects(*(model), *(data), _pinody->_q, _pinody->_qd);
    pinocchio::crba(*model, *data, _pinody->_q);
    pinocchio::computeJointJacobiansTimeVariation(*model, *data, _pinody->_q, _pinody->_qd);

    _H.setZero();
    _H = data->M;
    _C.setZero();
    _C = data->nle;

    _J[0].setZero();
    _J[1].setZero();
    _dJ[0].setZero();
    _dJ[1].setZero();
    pinocchio::getFrameJacobian(*model, *data, _pinody->joint_index[4], pinocchio::LOCAL, _J[0]);
    pinocchio::getFrameJacobian(*model, *data, _pinody->joint_index[9], pinocchio::LOCAL, _J[1]);
    pinocchio::getFrameJacobianTimeVariation(*model, *data, _pinody->joint_index[4], pinocchio::LOCAL, _dJ[0]);
    pinocchio::getFrameJacobianTimeVariation(*model, *data, _pinody->joint_index[9], pinocchio::LOCAL, _dJ[1]);

    int row_index = 0;
    for (int i = 0; i < 2;i++)
    {
        if (contact(i))
        {
            Js.block(row_index * 5, 0, 3, 25) = _J[i].block(0, 0, 3, 25);
            Js.block(row_index * 5 + 3, 0, 2, 25) = _J[i].block(4, 0, 2, 25);
            row_index++;
        }
        
    }

    MatX Js_T = Js.transpose();

    Eigen::HouseholderQR<Eigen::MatrixXd> qr = Js_T.householderQr();
    Eigen::MatrixXd Q_init = qr.householderQ();
    Eigen::MatrixXd R_init = qr.matrixQR().triangularView<Eigen::Upper>();

    _Qc.setZero(25, 5 * contact_num);
    _Qu.setZero(25, 25 - (5 * contact_num));
    _R.setZero(5 * contact_num, 5 * contact_num);
    _Qc = Q_init.block(0, 0, 25, 5 * contact_num);
    _Qu = Q_init.block(0, 5 * contact_num, 25, 25 - (5 * contact_num));
    _R = R_init.block(0, 0, 5 * contact_num, 5 * contact_num);

    _S.setZero(19, 25);
    _S.block(0, 6, 19, 19).setIdentity(19, 19);

    MatX A, b;
    A.setZero(25 - (5 * contact_num), 44);
    b.setZero(25 - (5 * contact_num), 1);
    A.block(0, 0, 25 - (5 * contact_num), 25) = _Qu.transpose() * (-_H);
    A.block(0, 25, 25 - (5 * contact_num), 19) = _Qu.transpose() * _S.transpose();
    b = _Qu.transpose() * _C;

    _eq_task[0] = new eq_Task(A, b, true);
    
}

void WBC::closure_constrain_task(VecInt2 contact)
{
    int contact_num = 0;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    MatX JF, dJF;
    JF.setZero(5 * contact_num, 25);
    dJF.setZero(5 * contact_num, 25);

    int row_index = 0;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i))
        {
            JF.block(row_index * 5, 0, 3, 25) = _J[i].block(0, 0, 5, 25);
            JF.block(row_index * 5 + 3, 0, 2, 25) = _J[i].block(4, 0, 2, 25);
            dJF.block(row_index * 5, 0, 3, 25) = _dJ[i].block(0, 0, 3, 25);
            dJF.block(row_index * 5 + 3, 0, 2, 25) = _dJ[i].block(4, 0, 2, 25);
            row_index++;
        }
    }

    MatX A, b;
    A.setZero(5 * contact_num, 44);
    b.setZero(5 * contact_num, 1);
    A.block(0, 0, 5 * contact_num, 25) = JF;
    b = -dJF * _pinody->_qd;

    _eq_task[1] = new eq_Task(A, b, true);
}

void WBC::desired_torso_motion_task(Vec2 ddr_xy)
{
    MatX A, b;
    A.setZero(2, 44);
    b.setZero(2, 1);
    A.block(0, 0, 2, 25) = _I_xy;
    b = ddr_xy;
    /*************************************/
    // A.setZero(18, 30);
    // b.setZero(18, 1);
    // A.block(0, 0, 18, 18).setIdentity(18, 18);
    /*************************************/

    _eq_task[2] = new eq_Task(A, b, true);
}

// swing_acc only contain swing foot acceleration, which means
// the swing cols of swing_acc must set zero when using this function
// swing foot in base coordinate
void WBC::swing_foot_motion_task(Vec3 swing_acc, VecInt2 contact, bool active)
{
    MatX A, b;
    Vec3 swing_acc_in_suc;
    int swing_num = 0;
    int swing_index = 0;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 0)
        {
            swing_index = i;
            swing_num++;
        }
    }
    if (swing_num == 0)
    {
        _eq_task[3] = new eq_Task(false);
        return;
    }
    A.setZero(swing_num * 5, 44);
    b.setZero(swing_num * 5, 1);

    Vec6 foot_twist_f; //coordinate: foot
    foot_twist_f.setZero();

    /*************  求ry  ****************/
    Mat4 T_foot;
    Mat6 X_foot;
    T_foot.setIdentity();
    X_foot.setIdentity();
    Eigen::Matrix<double, 10, 1> q_leg;
    for (int i = 0; i < 10; i++)
    {
        q_leg(i) = _dy->_q[i];
    }
    for (int i = 0 + 5 * swing_index; i < 5 + 5 * swing_index; i++)
    {
        X_foot = X_foot * _dy->_robot->X_dwtree[i];
        T_foot = T_foot * _dy->_robot->T_dwtree[i];
    }
   
    foot_twist_f.tail(3) =  swing_acc;
    Vec3 b_X_foot = T_foot.block(0, 0, 3, 1);
    Vec3 b_Z_terrain = _dy->_robot->Flt_Transform().transpose().block(0, 2, 3, 1);
    double xita = acos(b_X_foot.dot(b_Z_terrain) / b_X_foot.norm() * b_Z_terrain.norm());
    foot_twist_f(1) = 0 * (M_PI / 2 - xita);
    /*************************************/
    /*************  求rz  ****************/
    foot_twist_f(2) = 0 * (0 - q_leg(0 + swing_index * 5));
    /*************************************/

    Vec6 avp_foot2base = X_foot * _dy->_avp[4 + swing_index * 5];
    Vec6 base_twist_f = X_foot * foot_twist_f;

    MatX base_J_f = _dy->Cal_Geometric_Jacobain(4 + swing_index * 5, Coordiante::BASE);

    
    A.block(0, 6, 5, 19) = base_J_f.block(1, 0, 5, 19);
    b = base_twist_f.tail(5) - avp_foot2base.tail(5);
    // std::cout << "A: " << std::endl
    //           << A << std::endl;
    _eq_task[3] = new eq_Task(A, b, active);
}

void WBC::body_yaw_height_task(double yaw_acc, double height_acc)
{
    MatX A, b;
    A.setZero(2, 44);
    b.setZero(2, 1);
    Vec2 yaw_height;
    yaw_height << yaw_acc, height_acc;
    A.block(0, 0, 2, 25) = _I_yaw_height;
    b = yaw_height;

    _eq_task[5] = new eq_Task(A, b, true);
}

void WBC::body_roll_pitch_task(double roll_acc, double pitch_acc)
{
    MatX A, b;
    A.setZero(2, 44);
    b.setZero(2, 1);
    Vec2 roll_pitch;
    roll_pitch << roll_acc, pitch_acc;
    A.block(0, 0, 2, 25) = _I_roll_pitch;
    b = roll_pitch;

    _eq_task[4] = new eq_Task(A, b, true);
}

void WBC::torque_limit_task(VecInt2 contact, bool active)
{
    MatX A, b;
    A.setZero(44, 44);
    b.setZero(44, 1);
    Eigen::Matrix<double, 5, 5> I5;
    I5.setIdentity();
    // if (contact(0) == 0)
    // {
    //     A.block(25, 25, 5, 5) = I5;
    // }
    // else if (contact(1) == 0)
    // {
    //     A.block(30, 30, 5, 5) = I5;
    // }
    // else
    // {
        A.block(25, 25, 19, 19) = _I_torque;
    // }

    _eq_task[6] = new eq_Task(A, b, active);
}

// contact is the contact situation of foots   1: contact; 0: swing
void WBC::friction_cone_task(VecInt2 contact)
{
    int contact_num = 0;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    Mat4 T_lfoot = _pinody->_data->oMi[_pinody->joint_index[4]];
    Mat4 T_rfoot = _pinody->_data->oMi[_pinody->joint_index[9]];

    Mat3 s_R_foot[2];
    s_R_foot[0] = T_lfoot.block(0, 0, 3, 3);
    s_R_foot[1] = T_rfoot.block(0, 0, 3, 3);

    MatX s_R_f_ext;
    s_R_f_ext.setIdentity(contact_num * 5, contact_num * 5);
    int row_index = 0;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i))
        {
            s_R_f_ext.block(row_index * 5 + 2, row_index * 5 + 2, 3, 3) = s_R_foot[i];
            row_index++;
        }
    }
    MatX S_525_ext;
    S_525_ext.setZero(5 * contact_num, 5 * contact_num);
    for (int i = 0; i < contact_num; i++)
    {
        S_525_ext.block(i * 5, i * 5, 5, 5) = _S_525;
    }
    MatX B, beta;
    B.setZero(9 * contact_num, 5 * contact_num);
    beta.setZero(9 * contact_num, 1);
    for (int i = 0; i < contact_num; i++)
    {
        B.block(9 * i, 5 * i, 9, 5) = _Ffri;
        beta.block(9 * i, 0, 9, 1) = _fri_beta;
    }

    MatX brsR_1Qc_T;
    brsR_1Qc_T.setZero(9 * contact_num, 25);
    double R_det = _R.determinant();
    if (R_det *R_det < 0.1)
    {
        _R += Eigen::MatrixXd::Identity(_R.rows(), _R.cols()) * 0.1f;
    }
    brsR_1Qc_T = B * s_R_f_ext * S_525_ext * _R.inverse() * _Qc.transpose();

    MatX D, f;
    D.setZero(9 * contact_num, 44);
    f.setZero(9 * contact_num, 1);

    D.block(0, 0, 9 * contact_num, 25) = brsR_1Qc_T * (-_H);
    D.block(0, 25, 9 * contact_num, 19) = brsR_1Qc_T * _S.transpose();

    f = beta - brsR_1Qc_T * _C;

    _ineq_task = new ineq_Task(D, f);

}

void WBC::set_contact_frition(double fri)
{
    _frition = fri;
    _Ffri << 1, 0, 0, 0, 0,
        -1, 0, 0, 0, 0,
        0, 1, 0, 0, -_frition,
        0, -1, 0, 0, -_frition,
        0, 0, -1, 0, -_frition,
        0, 0, 1, 0, -_frition,
        0, 0, 0, -1, -_frition,
        0, 0, 0, 1, -_frition,
        0, 0, 0, 0, -1;
}

Vec12 WBC::inverse_dynamics(Vec18 qdd, Vec34 footforce, VecInt4 contact)
{
    /****************************************************************/
    // contact << 1, 1, 1, 1;
    MatX S_T;
    S_T.setZero(18, 12);
    S_T.block(6, 0, 12, 12).setIdentity(12, 12);
    _S.setZero(12, 18);
    _S.block(0, 6, 12, 12).setIdentity(12, 12);
    /****************************************************************/
    Vec12 torque;
    _H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(_H_fl);
    // MatX small_H;
    // small_H = _dy->Cal_Generalize_Inertial_Matrix_CRBA();
    // std::cout << "H: " << std::endl
    //           << small_H << std::endl;
    _C = _dy->Cal_Generalize_Bias_force_Flt(true);
    // MatX smallC = _dy->Cal_Generalize_Bias_force(true);
    // Vec12 errorC = smallC - _C.block(6, 0, 12, 1);
    // double norm_C = errorC.norm();
    // if (norm_C>0.01)
    // std::cout << "C_error: " << errorC.transpose() << std::endl;
    MatX K_temp, k_temp, lenda;
    K_temp = _dy->Cal_K_Flt(k_temp);
    int contact_num = 0;
    int row_index = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    // std::cout << "contact_num: " << contact_num << std::endl;
    _K.setZero(contact_num * 3, 18);
    _k.setZero(contact_num * 3, 1);
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            _K.block(3 * row_index, 0, 3, 18) = K_temp.block(3 * i, 0, 3, 18);
            _k.block(3 * row_index, 0, 3, 1) = k_temp.block(3 * i, 0, 3, 1);
            row_index++;
        }
    }
    MatX A, b;
    A.setZero(18, contact_num * 3 + 12);
    A.block(0, 0, 18, 12) = _S.transpose(); //_S.transpose()
    A.block(0, 12, 18, contact_num * 3) = _K.transpose();
    b = _H * qdd + _C;
    // std::cout << "Hqdd: " << std::endl <<qdd.transpose() * _H.transpose() << std::endl;
    // std::cout << "H: " << std::endl << _H << std::endl;
    MatX D, f;
    MatX bigR, bigF_fri;
    bigR.setZero(contact_num * 3, contact_num * 3);
    bigF_fri.setZero(contact_num * 5, contact_num * 3);
    f.setZero(contact_num * 5, 1);
    row_index = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            bigR.block(row_index * 3, row_index * 3, 3, 3) = _dy->_robot->_base->_fltjoint->_T_Wrd2Base.block(0, 0, 3, 3) * _dy->_ref_R_s[i];
            bigF_fri.block(5 * row_index, 3 * row_index, 5, 3) = _Ffri;
            // f(row_index * 6 + 5) = 80.0;
            row_index++;
        }
    }
    // std::cout << "f: " << f.transpose() << std::endl;
    D.setZero(contact_num * 5, contact_num * 3 + 12);
    D.block(0, 12, contact_num * 5, contact_num * 3) = bigF_fri * bigR;
    // std::cout << "yes" << std::endl;
    solve_QProblem(A, b, D, f);
    // std::cout << "yes" << std::endl;
    Eigen::FullPivLU<MatX> lu(A);
    // std::cout << "A_size: " << A.rows() << " " << A.cols() << std::endl;
    // VecX re1 = lu.solve(b);
    VecX re1 = _di;
    MatX null_A;
    VecX resultX1;
    // std::cout << "yes" << std::endl;
    if (!lu.isInvertible()) // not full rank
    {
        null_A = lu.kernel();
        solve_QProblem(null_A, -re1, D * null_A, D * re1 + f);
        resultX1 = null_A * _di + re1;
        re1 = resultX1;
        VecX frition_cone = D * resultX1 + f;
        // std::cout << " frition_cone " << frition_cone.transpose() << std::endl;
    }
    Vec12 footf;
    footf.setZero();
    for (int i = 0; i < 12; i++)
    {
        torque[i] = re1[i];
    }
    row_index = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            footf[3 * i] = re1[row_index * 3 + 12];
            footf[3 * i + 1] = re1[row_index * 3 + 13];
            footf[3 * i + 2] = re1[row_index * 3 + 14];
            row_index++;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            footf.block(i * 3, 0, 3, 1) = _dy->_ref_R_s[i] * footf.block(i * 3, 0, 3, 1);
        }
    }
    // std::cout << "frition_cone: " << std::endl
    //   << frition_cone.transpose() << std::endl;
    // std::cout << "frition_cone: " << std::endl
    //           << footf.transpose() * bigF_fri.transpose() << std::endl;
    // MatX error = A * _di - b;
    // Vec12 footuni = vec34ToVec12(footforce);
    // MatX force_temp = _K.transpose() * footf;
    // std::cout << "error: " << error.transpose() << std::endl;
    // std::cout << "KTlda: " << force_temp.transpose() << std::endl;
    // std::cout << "footfmy: " << footf.transpose() << std::endl;
    return torque;
}

void WBC::solve_HOproblem()
{

    MatX C_bar, d_bar, A_bar;
    MatX b_bar;
    C_bar.setIdentity(44, 44);
    d_bar.setZero(44, 1);
    b_bar.setZero(44, 1);
    int maxtask = -1;
    for (int i = 0; i < 7; i++)
    {
        if (_eq_task[i]->_active != true)
        {
            continue;
        }
        A_bar = _eq_task[i]->_A * C_bar;
        b_bar = _eq_task[i]->_b - _eq_task[i]->_A * d_bar;
        solve_QProblem(A_bar, b_bar, _ineq_task->_D * C_bar, _ineq_task->_D * d_bar + _ineq_task->_f); //
        d_bar = d_bar + C_bar * _di;
        Eigen::FullPivLU<MatX> lu(A_bar);
        if (lu.rank() >= A_bar.cols()) // if A_bar full rank
        {
            _qdd_torque = d_bar;
            maxtask = i;
            break;
        }
        C_bar = C_bar * lu.kernel();
    }
    _qdd_torque = d_bar;
    // std::cout << "max_task: " << maxtask << std::endl;
}

void WBC::solve_QProblem(MatX A, MatX b, MatX D, MatX f)
{
    int n = A.cols();
    int m = 0;
    int p = f.size(); // f.size()
    _G0.setZero(n, n);
    _g0.setZero(n, 1);
    _di.setZero(n, 1);
    _min_ident.setIdentity(n, n);
    _min_ident = _min_ident * 0.00000000001f;
    _G0 = A.transpose() * A + _min_ident;
    _g0 = A.transpose() * (-b);
    _CI = D;
    _ci0 = f;
    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            G[i][j] = _G0(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < p; ++j)
        {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        g0[i] = _g0(i);
    }

    for (int i = 0; i < p; ++i)
    {
        ci0[i] = _ci0(i);
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    // std::cout << "min value: " << value << std::endl;

    for (int i = 0; i < n; ++i)
    {
        _di[i] = x[i];
    }
}

VecX WBC::solve_QProblem_Ab(MatX A, MatX b)
{
    VecX result;
    int n = A.cols();
    int m = 0;
    int p = 0; // f.size()
    _G0.setZero(n, n);
    _g0.setZero(n, 1);
    _min_ident.setIdentity(n, n);
    _min_ident = _min_ident * 0.0001f;
    _G0 = A.transpose() * A + _min_ident;
    _g0 = A.transpose() * (-b);
    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            G[i][j] = _G0(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        g0[i] = _g0(i);
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    // // std::cout << "min value: " << value << std::endl;
    result.setZero(n, 1);
    for (int i = 0; i < n; ++i)
    {
        result(i) = x[i];
    }
    return result;
}
