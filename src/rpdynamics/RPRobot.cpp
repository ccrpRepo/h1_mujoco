#include "rpdynamics/RPRobot.h"
#include "rpdynamics/Spatial.h"

Robot::Robot(int NB, int NL)
:_NB(NB), _NL(NL)
{
    _base = new Base();

    _body = new Body[_NB]();
    _joint = new Joint[_NB]();
    _parent = new int[_NB]();
    X_dwtree = new Mat6[_NB]();
    X_uptree = new Mat6[_NB]();
    Xj = new Mat6[_NB]();
    Xq = new Mat6[_NB]();
    T_dwtree = new Mat4[_NB]();
    T_uptree = new Mat4[_NB]();
    Tj = new Mat4[_NB]();
    Tq = new Mat4[_NB]();

    _q = new double[_NB]();
    _dq = new double[_NB]();

    _lpjoint = new LoopJoint[_NL]();

    for (int i = 0; i < _NB; i++)
    {
        _parent[i] = i;
        X_dwtree[i] = Mat6::Identity(6, 6);
        X_uptree[i] = Mat6::Identity(6, 6);
        Xj[i] = Mat6::Identity(6, 6);
        Xq[i] = Mat6::Identity(6, 6);
        T_dwtree[i] = Mat4::Identity(4, 4);
        T_uptree[i] = Mat4::Identity(4, 4);
        Tj[i] = Mat4::Identity(4, 4);
        Tq[i] = Mat4::Identity(4, 4);
        // _q[i] = 0;
        // _dq[i] = 0;
    }

    for (int i = 0; i < NL; i++)
    {
        _lpjoint[i]._suc = -1;
        _lpjoint[i]._pre = -1;
        _lpjoint[i].Xp.setIdentity(6, 6);
        _lpjoint[i].Xs.setIdentity(6, 6);
    }
    _systick = getSystemTime();
}

// bool Robot::_isUpdate()
// {
//     long long sysnow = getSystemTime();
//     if ((sysnow - _systick) < 500)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

Robot::Robot(int NB)
    : _NB(NB)
{
    _NL = 0;
    _base = (Base *)malloc(sizeof(Base));
    _body = (Body *)malloc(sizeof(Body) * _NB);
    _joint = (Joint *)malloc(sizeof(Joint) * _NB);
    _parent = (int *)malloc(sizeof(int) * _NB);
    X_dwtree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    X_uptree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xj = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xq = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    T_dwtree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    T_uptree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tj = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tq = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    _q = (double *)malloc(sizeof(double) * _NB);
    _dq = (double *)malloc(sizeof(double) * _NB);

    for (int i = 0; i < _NB; i++)
    {
        _parent[i] = i;
        X_dwtree[i] = Mat6::Identity(6, 6);
        X_uptree[i] = Mat6::Identity(6, 6);
        Xj[i] = Mat6::Identity(6, 6);
        Xq[i] = Mat6::Identity(6, 6);
        T_dwtree[i] = Mat4::Identity(4, 4);
        T_uptree[i] = Mat4::Identity(4, 4);
        Tj[i] = Mat4::Identity(4, 4);
        Tq[i] = Mat4::Identity(4, 4);
    }
    _isUpdated = false;
    _systick = getSystemTime();
}

void Robot::Update_Model()
{

    if (!_isUpdated) // !_isUpdated
    {
        // std::cout << "update!!!" << std::endl;
        if (_base->get_BaseType() == BaseType::Floating)
        {
            Mat4 T = Flt_Transform();
            _base->_fltjoint->_T_Base2Wrd = T;
            AdjointT(T, _base->_fltjoint->_X_Base2Wrd);
            Mat3 R = T.block(0, 0, 3, 3);
            Vec3 p = T.block(0, 3, 3, 1);
            R.transposeInPlace();
            p = -R * p;
            T.setIdentity(4, 4);
            T.block(0, 0, 3, 3) = R;
            T.block(0, 3, 3, 1) = p;
            _base->_fltjoint->_T_Wrd2Base = T;
            AdjointT(T, _base->_fltjoint->_X_Wrd2Base);
            // Mat6 X_temp = _base->_fltjoint->_X_Wrd2Base * _base->_fltjoint->_X_Base2Wrd;
            // std::cout << "X_temp: " << std::endl
            //           << X_temp << std::endl;
        }
        else
        {
            _base->_fltjoint->_T_Base2Wrd.setIdentity();
            _base->_fltjoint->_T_Wrd2Base.setIdentity();
            _base->_fltjoint->_X_Base2Wrd.setIdentity();
            _base->_fltjoint->_X_Wrd2Base.setIdentity();
        }

        for (int i = 0; i < _NB; i++)
        {
            if (_joint[i]._jtype == JointType::RZ)
            {
                Xq[i] = Xrotz(_q[i]); // X_down
                Tq[i] = roz(_q[i]);  // T_down
            }
            else if (_joint[i]._jtype == JointType::RX)
            {
                Xq[i] = Xrotx(_q[i]); // X_down
                Tq[i] = rox(_q[i]);  // T_down
            }
            else if (_joint[i]._jtype == JointType::RY)
            {
                Xq[i] = Xroty(_q[i]); // X_down
                Tq[i] = roy(_q[i]);  // T_down
            }

            X_dwtree[i] = Xj[i] * Xq[i];
            T_dwtree[i] = Tj[i] * Tq[i];

            Mat3 R;
            Vec3 xyz;
            R = T_dwtree[i].block(0, 0, 3, 3).transpose();
            xyz = (-R) * T_dwtree[i].block(0, 3, 3, 1);
            Rp2T(R, xyz, T_uptree[i]);
            AdjointT(T_uptree[i], X_uptree[i]);
            // std::cout << i << ": " << std::endl
            //           << X_dwtree[i] << std::endl;
        }
    }
        
    _isUpdated = true;
}

void a1Robot::update_footEnd()
{
    Mat4 qicifoot_temp;
    Mat4 qicimat_temp;
    qicimat_temp = T_dwtree[0] * T_dwtree[1] * T_dwtree[2] * _lpjoint[0].Ts;
    qicifoot_temp.block(0, 0, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    qicimat_temp = T_dwtree[3] * T_dwtree[4] * T_dwtree[5] * _lpjoint[1].Ts;
    qicifoot_temp.block(0, 1, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    qicimat_temp = T_dwtree[6] * T_dwtree[7] * T_dwtree[8] * _lpjoint[2].Ts;
    qicifoot_temp.block(0, 2, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    qicimat_temp = T_dwtree[9] * T_dwtree[10] * T_dwtree[11] * _lpjoint[3].Ts;
    qicifoot_temp.block(0, 3, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    // qicifoot_temp = _base->_fltjoint->_T_Base2Wrd * qicifoot_temp;
    _endPfoot = qicifoot_temp.block(0, 0, 3, 4);
}

MatX Robot::Cal_Jacobian(int ib, Coordiante frame)
{
    MatX J;
    J.setZero(6, _NB); // initialize Jacobian matrix

    Update_Model();

    // initial k set
    int *k, *k_temp;
    k_temp = (int *)malloc(sizeof(int) * _NB);
    k = (int *)malloc(sizeof(int) * _NB);
    int j = ib;
    int num = 0;
    while (j >= 0)
    {
        k_temp[num] = j;
        j = _parent[j];
        num++;
    }
    for (int i = num - 1; i >= 0; --i)
    {
        k[num - 1 - i] = k_temp[i];
    }
    // Jacobian of body ib repesent at base frame
    if (frame == Coordiante::BASE) // base numbered -1
    {
        Mat6 X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num; i++)
        {
            X_down = X_down * X_dwtree[k[i]];
            J.block(0, k[i], 6, 1) = X_down * _joint[k[i]]._S_Body;
        }
    }
    // Jacobian of body ib repesent at body ib coordinate
    else if (frame == Coordiante::BODY)
    {
        Mat6 X_up;
        X_up.setIdentity(6, 6);
        J.block(0, k[num - 1], 6, 1) = _joint[k[num - 1]]._S_Body;
        for (int i = num - 1; i > 0; --i)
        {
            X_up = X_up * X_uptree[i];
            J.block(0, k[i - 1], 6, 1) = X_up * _joint[k[i - 1]]._S_Body;
        }
    }
    return J;
}

Eigen::Matrix<double,6,5> h1Robot::leg_Jacobian(Vec5 q, int endid)
{
    Eigen::Matrix<double, 6, 5> J;
    J.setZero(6, 5); // initialize Jacobian matrix

    int index;
    if (endid == 0) //left leg
    {
        index = 0;
    }
    else if (endid == 1) //right leg
    {
        index = 5;
    }
    Mat6 Xq_[5], X_dwtree_[5];

    for (int i = index; i < 5 + index; i++)
    {
        if (_joint[i]._jtype == JointType::RZ)
        {
            Xq_[i] = Xrotz(q[i]); // X_down
        }
        else if (_joint[i]._jtype == JointType::RX)
        {
            Xq_[i] = Xrotx(q[i]); // X_down
        }
        else if (_joint[i]._jtype == JointType::RY)
        {
            Xq_[i] = Xroty(q[i]); // X_down
        }
        X_dwtree_[i] = Xj[i] * Xq_[i];
    }
    Mat6 X_down;
    X_down.setIdentity(6, 6);
    for (int i = 0; i < 5; i++)
    {
        X_down = X_down * X_dwtree_[i];
        J.block(0, i, 6, 1) = X_down * _joint[i+index]._S_Body;
    }

    return J;
}

// q(0:3):quaternion
// q(4:6):xyz
Mat4 Robot::Flt_Transform()
{
    Mat4 T;
    Mat3 R;
    Vec3 xyz;
    xyz << _quat_xyz[4], _quat_xyz[5], _quat_xyz[6];
    // std::cout << "xyz: " << xyz.transpose() << std::endl;
    Eigen::Quaterniond qua(_quat_xyz[0],
                           _quat_xyz[1],
                           _quat_xyz[2],
                           _quat_xyz[3]); // w x y z
    qua.normalize();
    R = qua.matrix();
    // std::cout << "R_base: " <<std::endl<< R << std::endl;
    T.setIdentity(4, 4);
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = xyz;
    return T;
}

void a1Robot::witre_urdfData()
{
    Mat3 Ic[12];
    Vec3 com[12];
    double mass[12] = {0.696, 1.013, 0.226,
                       0.696, 1.013, 0.226,
                       0.696, 1.013, 0.226,
                       0.696, 1.013, 0.226};

    com[0]
        << -0.003311,
        -0.000635, 3.1e-05;
    Ic[0] << 0.000469246, 9.409e-06, -3.42e-07,
        9.409e-06, 0.00080749, 4.66e-07,
        -3.42e-07, 4.66e-07, 0.000552929;

    com[1] << -0.003237, 0.022327, -0.027326;
    Ic[1] << 0.005529065, -4.825e-06, 0.000343869,
        -4.825e-06, 0.005139339, -2.2448e-05,
        0.000343869, -2.2448e-05, 0.001367788;

    com[2] << 0.006435, 0.0, -0.107388;
    Ic[2] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    com[3] << -0.003311, 0.000635, 3.1e-05;
    Ic[3] << 0.000469246, -9.409e-06, -3.42e-07,
        -9.409e-06, 0.00080749, -4.66e-07,
        -3.42e-07, -4.66e-07, 0.000552929;

    com[4] << -0.003237, -0.022327, -0.027326;
    Ic[4] << 0.005529065, 4.825e-06, 0.000343869,
        4.825e-06, 0.005139339, 2.2448e-05,
        0.000343869, 2.2448e-05, 0.001367788;

    com[5] << 0.006435, 0.0, -0.107388;
    Ic[5] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    com[6] << 0.003311, -0.000635, 3.1e-05;
    Ic[6] << 0.000469246, -9.409e-06, 3.42e-07,
        -9.409e-06, 0.00080749, 4.66e-07,
        3.42e-07, 4.66e-07, 0.000552929;

    com[7] << -0.003237, 0.022327, -0.027326;
    Ic[7] << 0.005529065, -4.825e-06, 0.000343869,
        -4.825e-06, 0.005139339, -2.2448e-05,
        0.000343869, -2.2448e-05, 0.001367788;

    com[8] << 0.006435, 0.0, -0.107388;
    Ic[8] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    com[9] << 0.003311, 0.000635, 3.1e-05;
    Ic[9] << 0.000469246, 9.409e-06, 3.42e-07,
        9.409e-06, 0.00080749, -4.66e-07,
        3.42e-07, -4.66e-07, 0.000552929;

    com[10] << -0.003237, -0.022327, -0.027326;
    Ic[10] << 0.005529065, 4.825e-06, 0.000343869,
        4.825e-06, 0.005139339, 2.2448e-05,
        0.000343869, 2.2448e-05, 0.001367788;

    com[11] << 0.006435, 0.0, -0.107388;
    Ic[11] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    for (int i = 0; i < 12;i++)
    {
        _urdf->_urbody[i] = new urbody(com[i], Ic[i],mass[i]);
    }

    Mat3 Ic_base;
    Vec3 com_base;
    double mass_base = 6.0;
    com_base << 0, 0.0041, -0.0005;
    Ic_base << 0.0158533, 0, 0,
        0, 0.0377999, 0,
        0, 0, 0.0456542;
    _urdf->_urbase = new urbody(com_base, Ic_base, mass_base);

    Mat3 rpy[12];
    Vec3 xyz[12];
    JointType jt[12] = {JointType::RX, JointType::RY, JointType::RY,
                        JointType::RX, JointType::RY, JointType::RY,
                        JointType::RX, JointType::RY, JointType::RY,
                        JointType::RX, JointType::RY, JointType::RY};

    for (int i = 0; i < 12; i++)
    {
        rpy[i].setIdentity(3, 3);
    }
    xyz[0] << 0.1805, -0.047, 0;
    xyz[1] << 0, -0.0838, 0;
    xyz[2] << 0, 0, -0.2;
    xyz[3] << 0.1805, 0.047, 0;
    xyz[4] << 0, 0.0838, 0;
    xyz[5] << 0, 0, -0.2;
    xyz[6] << -0.1805, -0.047, 0;
    xyz[7] << 0, -0.0838, 0;
    xyz[8] << 0, 0, -0.2;
    xyz[9] << -0.1805, 0.047, 0;
    xyz[10] << 0, 0.0838, 0;
    xyz[11] << 0, 0, -0.2;

    for (int i = 0; i < 12; i++)
    {
        _urdf->_urjoint[i] = new urjoint(jt[i], rpy[i], xyz[i]);
    }

    JointType fltjt = JointType::FLOATING;
    _urdf->_urfltjoint = new urjoint(JointType::FLOATING, Mat3::Identity(), Vec3::Zero());
}

void a1Robot::build_a1()
{
    int parentset[12] = {-1, 0, 1, -1, 3, 4, -1, 6, 7, -1, 9, 10};
    // number the parent set
    for (int i = 0; i < _NB; i++)
    {
        _parent[i] = parentset[i];
    }

    // Floating base settings
    _base->set_BaseType(BaseType::Floating);
    _base->set_mcI(_urdf->_urbase);
    _base->_fltjoint = new FltJoint(_urdf->_urfltjoint);
    _base->_fltjoint->_T_Base2Wrd.setIdentity();
    _base->_fltjoint->_T_Wrd2Base.setIdentity();
    _base->_fltjoint->_X_Base2Wrd.setIdentity();
    _base->_fltjoint->_X_Wrd2Base.setIdentity();
    _quat_xyz[0] = 1.0f;
    _quat_xyz[1] = 0.0f;
    _quat_xyz[2] = 0.0f;
    _quat_xyz[3] = 0.0f;
    _quat_xyz[4] = 0.0f;
    _quat_xyz[5] = 0.0f;
    _quat_xyz[6] = 0.0f;

    for (int i = 0; i < _NB; i++)
    {
        _body[i].set_mcI(_urdf->_urbody[i]);
        _joint[i].set_type(_urdf->_urjoint[i]->_jt);
        _joint[i].set_rpy_xyz(_urdf->_urjoint[i]->_rpyMat,
                             _urdf->_urjoint[i]->_xyz);
        Rp2T(_joint[i]._rpyMat, _joint[i]._xyz, Tj[i]);
        AdjointT(Tj[i], Xj[i]);
        _q[i] = 0.0;
        Tq[i] = roz(_q[i]);
        T_dwtree[i] = Tj[i] * Tq[i];
    }

    Mat3 rpy_lp;
    Vec3 xyz_lp;
    rpy_lp << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz_lp << 0, 0, -0.2;
    for (int i = 0; i < _NL; i++)
    {
        _lpjoint[i]._pre = WORLD;
        _lpjoint[i].T.setZero(6, 3);
        _lpjoint[i].T << 0, 0, 0,
                            0, 0, 0,
                            0, 0, 0,
                            1, 0, 0,
                            0, 1, 0,
                            0, 0, 1;
        _lpjoint[i]._DOF = 3;
        Rp2T(rpy_lp, xyz_lp, _lpjoint[i].Ts);
        AdjointT(_lpjoint[i].Ts, _lpjoint[i].Xs);
        Mat3 R_t = rpy_lp.transpose();
        Vec3 xyz_t = (-R_t) * xyz_lp;
        Rp2T(R_t, xyz_t, _lpjoint[i].Ts_1);
        AdjointT(_lpjoint[i].Ts_1, _lpjoint[i].Xs_1);

        Rp2T(Mat3::Identity(), Vec3::Zero(), _lpjoint[i].Tp);
        Rp2T(Mat3::Identity(), Vec3::Zero(), _lpjoint[i].Tp_1);
        AdjointT(_lpjoint[i].Tp, _lpjoint[i].Xp);
        AdjointT(_lpjoint[i].Tp_1, _lpjoint[i].Xp_1);
    }
    _lpjoint[0]._suc = 2;
    _lpjoint[1]._suc = 5;
    _lpjoint[2]._suc = 8;
    _lpjoint[3]._suc = 11;

    // Update_Model();
    std::cout
        << std::endl
        << "Model building completed! NB = " << _NB << " NL = " << _NL << std::endl;
    std::cout << "The parent set is [ ";
    for (int i = 0; i < _NB; i++)
        std::cout << _parent[i] << " ";
    std::cout << "]" << std::endl;
}

void h1Robot::witre_urdfData()
{
    Vec6 Ic6_base;
    Vec3 com_base;
    double mass_base = 5.39;
    com_base << -0.0002, 4e-05, -0.04522;
    Ic6_base << 0.044582, 0.0082464, 0.049021, 8.7034E-05, -1.9893E-05, 4.021E-06;
    _urdf->_urbase = new urbody(com_base, Ic6_base, mass_base);

    Vec6 Ic6[19];
    Vec3 com[19];

    double mass[19] = {2.244, 2.232, 4.152, 1.721, 0.552448, // left leg
                       2.244, 2.232, 4.152, 1.721, 0.552448, // right leg
                       17.789,                               // torso
                       1.033, 0.793, 0.839, 0.669,           // left arm
                       1.033, 0.793, 0.839, 0.669            // right arm
    };

    // left leg
    com[0] << -0.04923, 0.0001, 0.0072;
    Ic6[0] << 0.0025731, 0.0030444, 0.0022883, 9.159E-06, -0.00051948, 1.949E-06;
    com[1] << -0.0058, -0.00319, -9e-05;
    Ic6[1] << 0.0020603, 0.0022482, 0.0024323, 3.2115E-05, 2.878E-06, -7.813E-06;
    com[2] << 0.00746, -0.02346, -0.08193;
    Ic6[2] << 0.082618, 0.081579, 0.0060081, -0.00066654, 0.0040725, 0.0072024;
    com[3] << -0.00136, -0.00512, -0.1384;
    Ic6[3] << 0.012205, 0.012509, 0.0020629, -6.8431E-05, 0.0010862, 0.00022549;
    com[4] << 0.048568, 0, -0.045609;
    Ic6[4] << 0.000159668, 0.002900286, 0.002805438, -0.000000005, 0.000141063, 0.000000014;

    // right leg
    com[5] << -0.04923, -0.0001, 0.0072;
    Ic6[5] << 0.0025731, 0.0030444, 0.0022883, -9.159E-06, -0.00051948, -1.949E-06;
    com[6] << -0.0058, 0.00319, -9e-05;
    Ic6[6] << 0.0020603, 0.0022482, 0.0024323, -3.2115E-05, 2.878E-06, 7.813E-06;
    com[7] << 0.00746, 0.02346, -0.08193;
    Ic6[7] << 0.082618, 0.081579, 0.0060081, 0.00066654, 0.0040725, -0.0072024;
    com[8] << -0.00136, 0.00512, -0.1384;
    Ic6[8] << 0.012205, 0.012509, 0.0020629, 6.8431E-05, 0.0010862, -0.00022549;
    com[9] << 0.048568, 0, -0.045609;
    Ic6[9] << 0.000159668, 0.002900286, 0.002805438, 0.000000005, 0.000141063, -0.000000014;

    // torso
    com[10] << 0.000489, 0.002797, 0.20484;
    Ic6[10] << 0.4873, 0.40963, 0.12785, -0.00053763, 0.0020276, -0.00074582;

    // left arm
    com[11] << 0.005045, 0.053657, -0.015715;
    Ic6[11] << 0.0012985, 0.00087279, 0.00097338, -1.7333E-05, 8.683E-06, 3.9656E-05;
    com[12] << 0.000679, 0.00115, -0.094076;
    Ic6[12] << 0.0015742, 0.0016973, 0.0010183, 2.298E-06, -7.2265E-05, -6.3691E-05;
    com[13] << 0.01365, 0.002767, -0.16266;
    Ic6[13] << 0.003664, 0.0040789, 0.00066383, -1.0671E-05, 0.00034733, 7.0213E-05;
    com[14] << 0.15908, - 0.000144, -0.015776;
    Ic6[14] << 0.00042388, 0.0060062, 0.0060023, -3.6086E-05, 0.00029293, 4.664E-06;

    // right arm
    com[15] << 0.005045, -0.053657, -0.015715;
    Ic6[15] << 0.0012985, 0.00087279, 0.00097338, 1.7333E-05, 8.683E-06, -3.9656E-05;
    com[16] << 0.000679, -0.00115, -0.094076;
    Ic6[16] << 0.0015742, 0.0016973, 0.0010183, -2.298E-06, -7.2265E-05, 6.3691E-05;
    com[17] << 0.01365, -0.002767, -0.16266;
    Ic6[17] << 0.003664, 0.0040789, 0.00066383, 1.0671E-05, 0.00034733, -7.0213E-05;
    com[18] << 0.15908, 0.000144, -0.015776;
    Ic6[18] << 0.00042388, 0.0060062, 0.0060023, 3.6086E-05, 0.00029293, -4.664E-06;

    for (int i = 0; i < 19; i++)
    {
        _urdf->_urbody[i] = new urbody(com[i], Ic6[i], mass[i]);
    }

    Mat3 rpy[19];
    Vec3 xyz[19];
    JointType jt[19] = {JointType::RZ, JointType::RX, JointType::RY, JointType::RY, JointType::RY,
                        JointType::RZ, JointType::RX, JointType::RY, JointType::RY, JointType::RY,
                        JointType::RZ,
                        JointType::RY, JointType::RX, JointType::RZ, JointType::RY,
                        JointType::RY, JointType::RX, JointType::RZ, JointType::RY};

    for (int i = 0; i < 19; i++)
    {
        rpy[i].setIdentity(3, 3);
    }

    Eigen::Quaterniond quad1(0.976296, 0.216438, 0, 0);
    rpy[11] = quad1.matrix();
    Eigen::Quaterniond quad2(0.976296, -0.216438, 0, 0);
    rpy[12] = quad2.matrix();
    Eigen::Quaterniond quad3(0.976296, -0.216438, 0, 0);
    rpy[15] = quad3.matrix();
    Eigen::Quaterniond quad4(0.976296, 0.216438, 0, 0);
    rpy[16] = quad4.matrix();

    xyz[0]<< 0, 0.0875, -0.1742;
    xyz[1] << 0.039468, 0, 0;
    xyz[2] << 0, 0.11536, 0;
    xyz[3] << 0, 0, -0.4;
    xyz[4] << 0, 0, -0.4;

    xyz[5] << 0, -0.0875, -0.1742;
    xyz[6] << 0.039468, 0, 0;
    xyz[7] << 0, -0.11536, 0;
    xyz[8] << 0, 0, -0.4;
    xyz[9] << 0, 0, -0.4;

    xyz[10] << 0, 0, 0;

    xyz[11] << 0.0055, 0.15535, 0.42999;
    xyz[12] << -0.0055, 0.0565, -0.0165;
    xyz[13] << 0, 0, -0.1343;
    xyz[14] << 0.0185, 0, -0.198;

    xyz[15] << 0.0055, -0.15535, 0.42999;
    xyz[16] << -0.0055, -0.0565, -0.0165;
    xyz[17] << 0, 0, -0.1343;
    xyz[18] << 0.0185, 0, -0.198;

    for (int i = 0; i < 19; i++)
    {
        _urdf->_urjoint[i] = new urjoint(jt[i], rpy[i], xyz[i]);
    }

    JointType fltjt = JointType::FLOATING;
    _urdf->_urfltjoint = new urjoint(JointType::FLOATING, Mat3::Identity(), Vec3::Zero());
}

void h1Robot::build_h1()
{
    int parentset[19] = {-1,0,1,2,3, 
                         -1,5,6,7,8,
                         -1,
                          10,11,12,13,
                          10,15,16,17};
    // number the parent set
    for (int i = 0; i < _NB; i++)
    {
        _parent[i] = parentset[i];
    }

    //Floating base settings
    _base->set_BaseType(BaseType::Floating);
    _base->set_mcI(_urdf->_urbase);
    _base->_fltjoint = new FltJoint(_urdf->_urfltjoint);
    _base->_fltjoint->_T_Base2Wrd.setIdentity();
    _base->_fltjoint->_T_Wrd2Base.setIdentity();
    _base->_fltjoint->_X_Base2Wrd.setIdentity();
    _base->_fltjoint->_X_Wrd2Base.setIdentity();
    _quat_xyz[0] = 1.0f;
    _quat_xyz[1] = 0.0f;
    _quat_xyz[2] = 0.0f;
    _quat_xyz[3] = 0.0f;
    _quat_xyz[4] = 0.0f;
    _quat_xyz[5] = 0.0f;
    _quat_xyz[6] = 0.0f;

    for (int i = 0; i < _NB; i++)
    {
        _body[i].set_mcI(_urdf->_urbody[i]);
        _joint[i].set_type(_urdf->_urjoint[i]->_jt);
        _joint[i].set_rpy_xyz(_urdf->_urjoint[i]->_rpyMat,
                              _urdf->_urjoint[i]->_xyz);
        Rp2T(_joint[i]._rpyMat, _joint[i]._xyz, Tj[i]);
        AdjointT(Tj[i], Xj[i]);
        _q[i] = 0.0;
        Tq[i] = roz(_q[i]);
        T_dwtree[i] = Tj[i] * Tq[i];
    }

    // loop joint settings
    Mat3 rpy_lp;
    Vec3 xyz_lp;
    rpy_lp << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz_lp << 0, 0, -0.07;

    for (int i = 0; i < _NL; i++)
    {
        _lpjoint[i]._pre = WORLD;
        _lpjoint[i].T.setZero(6, 5);
        _lpjoint[i].T << 0, 0, 0, 0, 0,
                         1, 0, 0, 0, 0,
                         0, 1, 0, 0, 0,
                         0, 0, 1, 0, 0,
                         0, 0, 0, 1, 0,
                         0, 0, 0, 0, 1;
        _lpjoint[i]._DOF = 1;
        Rp2T(rpy_lp, xyz_lp, _lpjoint[i].Ts);
        AdjointT(_lpjoint[i].Ts, _lpjoint[i].Xs);
        Mat3 R_t = rpy_lp.transpose();
        Vec3 xyz_t = (-R_t) * xyz_lp;
        Rp2T(R_t, xyz_t, _lpjoint[i].Ts_1);
        AdjointT(_lpjoint[i].Ts_1, _lpjoint[i].Xs_1);

        Rp2T(Mat3::Identity(), Vec3::Zero(), _lpjoint[i].Tp);
        Rp2T(Mat3::Identity(), Vec3::Zero(), _lpjoint[i].Tp_1);
        AdjointT(_lpjoint[i].Tp, _lpjoint[i].Xp);
        AdjointT(_lpjoint[i].Tp_1, _lpjoint[i].Xp_1);
    }

    _lpjoint[0]._suc = 4;
    _lpjoint[1]._suc = 9;

    // Update_Model();
    std::cout
        << std::endl
        << "Model building completed! NB = " << _NB << " NL = " << _NL << std::endl;
    std::cout << "The parent set is [ ";
    for (int i = 0; i < _NB; i++)
        std::cout << _parent[i] << " ";
    std::cout << "]" << std::endl;
}

// inverse kinematic off coordinate pelvis
// endposyawpitch: pos(0~2) yaw(3) pitch(4)
// endid: left_leg(0) right_leg(1) left_arm(2) right_arm(3)
Vec5 h1Robot::getQ(Vec5 endposyawpitch, int endid)
{
    Update_Model();
    Vec5 qd;
    qd.setZero();

    Vec3 pos = endposyawpitch.head(3);
    double yaw = endposyawpitch(3);
    double pitch = endposyawpitch(4);

    qd(0) = yaw;

    float sideSign;
    if (endid == 0) // left leg
    {
        sideSign = 1;
    }
    else if (endid == 1) // right leg
    {
        sideSign = -1;
    }

    double y_hip2pelvis = 0.0875;
    Vec3 hip2pelvis_offset;
    hip2pelvis_offset << 0, y_hip2pelvis * sideSign, -0.1742;

    Vec3 hiproll2yaw;
    hiproll2yaw << 0.039468, 0, 0;

    /******************3DoF Calculate*************************/
    Vec3 dog_pos = pos - rox(yaw).block(0, 0, 3, 3) * hiproll2yaw - hip2pelvis_offset;

    double abadLinkLength = 0.11536;
    double hipLinkLength = 0.4;
    double kneeLinkLength = 0.4;

    float q1, q2, q3;
    Vec3 qResult;
    float px, py, pz;
    float b2y, b3z, b4z, a, b, c;

    px = dog_pos(0);
    py = dog_pos(1);
    pz = dog_pos(2);

    b2y = abadLinkLength * sideSign;
    b3z = -hipLinkLength;
    b4z = -kneeLinkLength;
    a = abadLinkLength;
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2));                // distance between shoulder and footpoint

    // q1 = q1_ik(py, pz, b2y);
    float L = sqrt(pow(py, 2) + pow(pz, 2) - pow(b2y, 2));
    q1 = atan2(pz * b2y + py * L, py * b2y - pz * L);

    // q3 = q3_ik(b3z, b4z, b);
    float temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2)) / (2 * fabs(b3z * b4z));
    if (temp > 1)
        temp = 1;
    if (temp < -1)
        temp = -1;
    q3 = acos(temp);
    q3 = (M_PI - q3); // 0~180

    // q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);
    float a1, a2, m1, m2;

    a1 = py * sin(q1) - pz * cos(q1);
    a2 = px;
    m1 = b4z * sin(q3);
    m2 = b3z + b4z * cos(q3);
    q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

    qd(1) = q1;
    qd(2) = q2;
    qd(3) = q3;

    /*******************************************************/
    Mat4 Tq[5];
    Mat6 Xq[5];
    Mat4 T_dwtree[5];
    Vec5 q_cur;
    int k[5];
    if (endid == 0) // left leg
    {
        k[0] = 0;
        k[1] = 1;
        k[2] = 2;
        k[3] = 3;
        k[4] = 4;
    }
    if (endid == 1) // right leg
    {
        k[0] = 5;
        k[1] = 6;
        k[2] = 7;
        k[3] = 8;
        k[4] = 9;
    }
    for (int i = 0; i < 4; i++)
    {
        if (_joint[k[i]]._jtype == JointType::RZ)
        {
            Tq[i] = roz(qd(i)); // T_down
        }
        else if (_joint[k[i]]._jtype == JointType::RX)
        {
            Tq[i] = rox(qd(i)); // T_down
        }
        else if (_joint[k[i]]._jtype == JointType::RY)
        {
            Tq[i] = roy(qd(i)); // T_down
        }
        T_dwtree[i] = Tj[k[i]] * Tq[i];
    }
    Mat4 T_cur;
    T_cur.setIdentity(4, 4);

    for (int i = 0; i < 4; i++)
    {
        T_cur = T_cur * T_dwtree[i];
    }

    // 求平底足法向向量
    Vec3 norm_planfoot(sin(pitch), 0, cos(pitch));
    // 小腿坐标系x轴方向向量
    Vec3 calf_xaxis = T_cur.block(0, 0, 3, 1);

    // 两向量夹角
    double xita = acos(norm_planfoot.dot(calf_xaxis) / norm_planfoot.norm() * calf_xaxis.norm());

    qd(4) = M_PI / 2 - xita;

    if (qd(0) > 0.43)
        qd(0) = 0.429;
    else if (qd(0) < -0.43)
        qd(0) = -0.429;

    if (qd(1) > 0.86)
        qd(1) = 0.86;
    else if (qd(1) < -0.86)
        qd(1) = -0.86;

    if (qd(2) > 1.57)
        qd(2) = 1.569;
    else if (qd(2) < -1.57)
        qd(2) = -1.569;

    if (qd(3) > 2.05)
        qd(3) = 2.049;
    else if (qd(3) < -0.26)
        qd(3) = -0.259;

    if (qd(4) > 0.52)
        qd(4) = 0.519;
    else if (qd(4) < -0.87)
        qd(4) = -0.869;

    return qd;
}

//return footend of coordinate pelvis
Mat52 h1Robot::get_footEnd()
{
    Mat52 footend;
    footend.setZero(5, 2);

    Update_Model();
    Mat4 T_lf; //left foot
    T_lf.setIdentity(4, 4);
    for (int i = 0; i < 5;i++)
    {
        T_lf = T_lf * T_dwtree[i];
    }
    Mat4 T_rf; // right foot
    T_rf.setIdentity(4, 4);
    for (int i = 5; i < 10; i++)
    {
        T_rf = T_rf * T_dwtree[i];
    }

    // 足坐标系x轴方向向量
    Vec3 lf_xaxis = T_lf.block(0, 0, 3, 1);
    Vec3 rf_xaxis = T_rf.block(0, 0, 3, 1);

    Vec3 xais(1, 0, 0);
    // 两向量夹角
    double xita_lf = acos(xais.dot(lf_xaxis) / xais.norm() * lf_xaxis.norm());
    double xita_rf = acos(xais.dot(rf_xaxis) / xais.norm() * rf_xaxis.norm());

    footend.col(0).head(3) = T_lf.block(0, 3, 3, 1); // left foot pos
    footend.col(0).tail(2) = Vec2(_q[0], xita_lf);   // left foot yaw and pitch
    footend.col(1).head(3) = T_rf.block(0, 3, 3, 1); // right foot pos
    footend.col(1).tail(2) = Vec2(_q[5], xita_rf);   // left foot yaw and pitch

    return footend;
}

// endid = 0 left leg
// endid = 1 right leg
Vec5 h1Robot::getQd(Vec5 endposyawpitch, int endid, Vec6 twist)
{
    Vec5 q = getQ(endposyawpitch, endid);
    Eigen::Matrix<double, 6, 5> J; // leg jacobian
    J = leg_Jacobian(q,endid);

    Eigen::Matrix<double, 5, 6> J_T = J.transpose();
    Eigen::Matrix<double, 5, 5> J_TJ = J_T * J;
    Eigen::Matrix<double, 5, 6> J_pesuinv = J_TJ.inverse() * J_T;

    Vec5 dq = J_pesuinv * twist;

    return dq;
}

// endid = 0 left leg
// endid = 1 right leg
Vec5 h1Robot::getTau(Vec5 q, int endid, Vec6 force)
{
    Vec5 tau;
    tau.setZero();

    Eigen::Matrix<double, 6, 5> J; // leg jacobian
    J = leg_Jacobian(q, endid);

    tau = J.transpose() * force;

    return tau;
}

Vec6 h1Robot::getFootVelocity(int endid)
{
    Vec5 q,dq;
    int index = endid * 5;
    for (int i = index; i < 5 + index; i++)
    {
        q(i) = _q[i];
        dq(i) = _dq[i];
    }

    Eigen::Matrix<double, 6, 5> J = leg_Jacobian(q, endid);

    Vec6 twist = J * dq;

    return twist;
}

Vec6 h1Robot::getFeet2BVelocity(int endid, Coordiante frame)
{
    Vec6 twist = getFootVelocity(endid);//in pelivs frame

    if(frame == Coordiante::BASE)
    {
        return twist;
    }
    else if(frame == Coordiante::INERTIAL)
    {
        Mat4 T = Flt_Transform();
        Mat6 X;
        AdjointT(T, X);
        return X * twist;
    }
}