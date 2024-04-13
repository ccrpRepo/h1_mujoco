#include "control/WBCController.h"

void WBC::dynamics_consistence_task(VecInt2 contact)
{
    int contact_num = 0;
    int row_index = 0;
    MatX K_temp, k_temp;
    // calculate general inertial matrix
    _H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(_H_fl);
    // std::cout << "_H: " << std::endl
    //           << _H << std::endl;
    // calculate general bias force
    _C = _dy->Cal_Generalize_Bias_force_Flt(true);
    // std::cout << "_C: " << std::endl
    //           << _C.transpose() << std::endl;
    // calculate constrain matrix K and k
    K_temp = _dy->Cal_K_Flt(k_temp);
    // std::cout << "K_temp: " << std::endl
    //           << K_temp << std::endl;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    _K.setZero(contact_num * 5, 25);
    _k.setZero(contact_num * 5, 1);
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 1)
        {
            _K.block(5 * row_index, 0, 5, 25) = K_temp.block(5 * i, 0, 5, 25);
            _k.block(5 * row_index, 0, 5, 1) = k_temp.block(5 * i, 0, 5, 1);
            row_index++;
        }
    }
    // calculate nullspace matrix of K
    Eigen::FullPivLU<MatX> lu(_K);
    _G = lu.kernel();

    // dynamics constrain A and b
    MatX A, b;
    int cols, rows;
    rows = _G.rows(); // G : rows x cols   G^T : cols x rows
    cols = _G.cols();
    // std::cout << "size: " << rows << ", " << cols << std::endl;
    A.setZero(cols, 44);
    b.setZero(cols, 1);
    _S.setZero(19, 25);
    _S.block(0, 6, 19, 19).setIdentity(19, 19);
    A.block(0, 0, cols, 25) = _G.transpose() * _H;
    A.block(0, 25, cols, 19) = -_G.transpose() * _S.transpose();

    b = -_G.transpose() * _C;

    _eq_task[0] = new eq_Task(A, b, true);

    // std::cout << "A: " << std::endl << A << std::endl;
    // std::cout << "b: " << b.transpose() << std::endl;
}

void WBC::closure_constrain_task()
{
    MatX A, b;
    int cols, rows;
    rows = _K.rows(); // K : rows x cols
    cols = _K.cols();
    A.setZero(rows, 44);
    A.block(0, 0, rows, 25) = _K;
    b = _k;

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
void WBC::swing_foot_motion_task(Vec3 swing_acc, VecInt2 contact)
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
    T_foot.setIdentity();
    Eigen::Matrix<double, 10, 1> q_leg;
    for (int i = 0; i < 10; i++)
    {
        q_leg(i) = _dy->_q[i];
    }
    for (int i = 0 + 5 * swing_index; i < 5 + 5 * swing_index; i++)
    {
        T_foot = T_foot * _dy->_robot->T_dwtree[i];
    }
    foot_twist_f.tail(3) = T_foot.block(0, 0, 3, 3).transpose() * swing_acc;
    Vec3 b_X_foot = T_foot.block(0, 0, 3, 1);
    Vec3 b_Z_terrain = _dy->_robot->Flt_Transform().transpose().block(0, 2, 3, 1);
    double xita = acos(b_X_foot.dot(b_Z_terrain) / b_X_foot.norm() * b_Z_terrain.norm());
    foot_twist_f(1) = 10 * (M_PI / 2 - xita);
    /*************************************/
    /*************  求rz  ****************/
    foot_twist_f(2) = 10 * (0 - q_leg(0 + swing_index * 5));
    /*************************************/

    Vec6 avp_foot2base = _dy->_ref_X_s[swing_index] * _dy->_avp[4 + swing_index * 5];
    Vec6 base_twist_f = _dy->_ref_X_s[swing_index] * foot_twist_f;

    MatX base_J_f = _dy->Cal_Geometric_Jacobain(4 + swing_index + 5, Coordiante::BASE);

    A.block(0, 6, 5, 19) = base_J_f.block(1, 0, 5, 19);
    b = foot_twist_f.tail(5) - avp_foot2base.tail(5);

    _eq_task[3] = new eq_Task(A, b, false);
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

    _eq_task[4] = new eq_Task(A, b, true);
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

    _eq_task[5] = new eq_Task(A, b, true);
}

void WBC::torque_limit_task()
{
    MatX A, b;
    A.setZero(19, 44);
    b.setZero(19, 1);
    A.block(0, 25, 19, 19) = _I_torque;

    _eq_task[6] = new eq_Task(A, b, false);
}

// contact is the contact situation of foots   1: contact; 0: swing
void WBC::friction_cone_task(VecInt2 contact)
{
    MatX D, f;
    int row_index = 0;
    MatX B;
    MatX ref_R_s_ext;
    int contact_num = 0;
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }

    D.setZero(9 * contact_num, 44);
    f.setZero(9 * contact_num, 1);
    B.setZero(9 * contact_num, 5 * contact_num);
    ref_R_s_ext.setIdentity(contact_num * 5, contact_num * 5);
    for (int i = 0; i < contact_num; i++)
    {
        B.block(9 * i, 5 * i, 9, 5) = _Ffri;
    }
    Mat3 Iden;
    Iden.setIdentity(3, 3);
    for (int i = 0; i < 2; i++)
    {
        if (contact(i) == 1)
        {
            ref_R_s_ext.block(row_index * 5 + 2, row_index * 5 + 2, 3, 3) = _dy->_ref_R_s[i]; //_dy->_ref_R_s[i];
            row_index++;
        }
    }
    MatX KK_T = _K * _K.transpose();
    MatX K_leftinv = KK_T.inverse() * _K;
    MatX D_temp;
    D_temp.setZero(25, 44);
    D_temp.block(0, 0, 25, 25) = -_H;
    D_temp.block(0, 25, 25, 19) = _S.transpose();
    MatX BRK_inv, Fri_beta;
    Fri_beta.setZero(9 * contact_num, 1);
    Fri_beta.block(0,0,9,1) = _fri_beta;
    if (contact_num == 2)
    {
        Fri_beta.block(9, 0, 9, 1) = _fri_beta;
    }
    BRK_inv.setZero(9 * contact_num, 25);
    BRK_inv = B * ref_R_s_ext * K_leftinv;
    D = BRK_inv * D_temp;
    f = Fri_beta - BRK_inv * _C;
    _D = D;
    _f = f;

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
    _min_ident = _min_ident * 0.0001f;
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
