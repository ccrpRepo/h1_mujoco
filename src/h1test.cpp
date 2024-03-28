#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>

#include "rpdynamics/dynamics.h"
#include "interface/IOmujoco.h"

using namespace std;

std::string _robot_name;

mjModel *m;
mjData *d;      // MuJoCo data
mjvCamera cam;  // abstract camera
mjvOption opt;  // visualization options     此结构包含启用和禁用各种元素可视化的选项。
mjvScene scn;   // abstract scene            该结构包含在 OpenGL 中渲染 3D 场景所需的一切。
mjrContext con; // custom GPU context        该结构包含自定义 OpenGL 渲染上下文，以及上传到 GPU 的所有 OpenGL 资源的 ID。
GLFWwindow *window;

// mouse interaction
bool button_left;
bool button_middle;
bool button_right;
double lastx;
double lasty;

bool running = true;

void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;
    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;
    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}

void Init_window()
{
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "CRP", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);              // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150); // model-specific context
    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    double arr_view[] = {90, -90, 10, 0.000000, 0.000000, 0.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
}

int main(int argc, char** argv)
{
    IOinterface *ioInter;
    CtrlPlatform ctrlPlat;

    std::cout << "The control interface for mujoco simulation" << std::endl;
    _robot_name = "humanoid_H1";
    std::cout << "robot_name: " << _robot_name << std::endl;

    char filename[] = "/home/crp/git/h1_sys/robot/scene.xml";
    char error[1000] = "Could not load binary model";
    lastx = 0;
    lasty = 0;
    button_left = false;
    button_middle = false;
    button_right = false;
    m = mj_loadXML(filename, 0, error, 1000);
    if (!m)
    {
        mju_error_s("Load model error: %s", error);
        cout << "DoF: " << endl;
    }

    d = mj_makeData(m);
    

    
    Init_window();
    ioInter = new IOmujoco(d,m);
    ctrlPlat = CtrlPlatform::MUJOCO;

    h1Robot *h1 = new h1Robot();
    Dynamics *dy = new Dynamics(h1);

    // // xyz
    // d->qpos[0] = 0;
    // d->qpos[1] = 0;
    // d->qpos[2] = 1.5;
    // // quat
    // d->qpos[3] = 1;
    // d->qpos[4] = 0;
    // d->qpos[5] = 0;
    // d->qpos[6] = 0;
    // left leg
    d->qpos[7 - 7] = 0; // -0.43 ~ 0.43
    d->qpos[8 - 7] = 0; // -0.43 ~ 0.43
    d->qpos[9 - 7] = -0.4; // -1.57 ~ 1.57
    d->qpos[10 - 7] = 0.8; // -0.26 ~ 2.05
    d->qpos[11 - 7] = -0.4; // -0.87 ~ 0.52
    // right leg
    d->qpos[12 - 7] = 0; // -0.43 ~ 0.43
    d->qpos[13 - 7] = 0; // -0.43 ~ 0.43
    d->qpos[14 - 7] = -0.4; // -1.57 ~ 1.57
    d->qpos[15 - 7] = 0.8;  // -0.26 ~ 2.05
    d->qpos[16 - 7] = -0.4; // -0.87 ~ 0.52
    // torso
    d->qpos[17 - 7] = 0; // -2.35 ~ 2.35
    // left arm
    d->qpos[18 - 7] = 0; // -2.87 ~ 2.87
    d->qpos[19 - 7] = 0; // -0.34 ~ 3.11
    d->qpos[20 - 7] = 0; // -1.3 ~ 4.45
    d->qpos[21 - 7] = 0; // -1.25 ~ 2.61
    // right arm
    d->qpos[22 - 7] = 0; // -2.87 ~ 2.87
    d->qpos[23 - 7] = 0; // -0.34 ~ 3.11
    d->qpos[24 - 7] = 0; // -1.3 ~ 4.45
    d->qpos[25 - 7] = 0; // -1.25 ~ 2.61

    bool first_run = true;
    Vec5 pos_start_l, pos_end_l, pos_cur_l;
    Vec5 pos_start_r, pos_end_r, pos_cur_r;
    pos_start_l.setZero();
    pos_start_r.setZero();
    while (running)
    {
        running = !glfwWindowShouldClose(window);
        double phase = 0;
        double swing_time = 2;
        MatX qd_left,qd_righ;
        qd_left.setZero(5, 1);
        qd_righ.setZero(5, 1);
        Mat4 Te_l,Te_r;
        mjtNum simstart = d->time;
        while ((d->time - simstart) < 1.0 / 60.0)
        {
            /* --------------------------*********TEST CODE BEGIN*********----------------------------*/
            // xyz
            // d->qpos[0] = 0;
            // d->qpos[1] = 0;
            // d->qpos[2] = 1.5;
            // // quat
            // d->qpos[3] = 1;
            // d->qpos[4] = 0;
            // d->qpos[5] = 0;
            // d->qpos[6] = 0;
            // // left leg
            // d->qpos[7] = qd_left(0); // -0.43 ~ 0.43
            // d->qpos[8] = qd_left(1); // -0.43 ~ 0.43
            // d->qpos[9] = qd_left(2); // -1.57 ~ 1.57
            // d->qpos[10] = qd_left(3); // -0.26 ~ 2.05
            // d->qpos[11] = qd_left(4); // -0.87 ~ 0.52
            // // right leg
            // d->qpos[12] = qd_righ(0); // -0.43 ~ 0.43
            // d->qpos[13] = qd_righ(1); // -0.43 ~ 0.43
            // d->qpos[14] = qd_righ(2); // -1.57 ~ 1.57
            // d->qpos[15] = qd_righ(3); // -0.26 ~ 2.05
            // d->qpos[16] = qd_righ(4); // -0.87 ~ 0.52
            // torso
            d->qpos[17-7] = 0; // -2.35 ~ 2.35
            // left arm
            d->qpos[18 - 7] = 0; // -2.87 ~ 2.87
            d->qpos[19 - 7] = 0; // -0.34 ~ 3.11
            d->qpos[20 - 7] = 0; // -1.3 ~ 4.45
            d->qpos[21 - 7] = 0; // -1.25 ~ 2.61
            // right arm
            d->qpos[22 - 7] = 0; // -2.87 ~ 2.87
            d->qpos[23 - 7] = 0; // -0.34 ~ 3.11
            d->qpos[24 - 7] = 0; // -1.3 ~ 4.45
            d->qpos[25 - 7] = 0; // -1.25 ~ 2.61

            for (int i = 0; i < 19; i++)
            {
                dy->_robot->_q[i] = d->qpos[i];
            }
            dy->_robot->Update_Model();

            Te_l.setIdentity(4, 4);
            Te_r.setIdentity(4, 4);
            for (int i = 0; i < 5; i++)
            {
                Te_l = Te_l * dy->_robot->T_dwtree[i];
            }
            for (int i = 5; i < 10; i++)
            {
                Te_r = Te_r * dy->_robot->T_dwtree[i];
            }
            // cout << "Te: " << endl
            //      << Te << endl;
            if (first_run)
            {
                pos_start_l << 0, 0.2, -0.85, 0, 0;
                pos_start_r << 0, -0.2, -0.85, 0, 0;
                first_run = false;
            }
            phase = (sin(d->time) + 1) / 2;

            pos_end_l << 0.3, 0.2, -0.6, 0.3, 0.5;
            pos_end_r << 0.3, -0.2, -0.6, -0.3, 0.5;
            pos_cur_l = (1 - phase) * pos_start_l + phase * pos_end_l;
            pos_cur_r = (1 - phase) * pos_start_r + phase * pos_end_r;

            Vec5 endposyawpitch_left, endposyawpitch_righ;
            endposyawpitch_left = pos_cur_l;
            endposyawpitch_righ = pos_cur_r;
            // cout << pos_cur_l.transpose() << endl;
            qd_left = dy->Cal_inverse_kinematic_Analytical(endposyawpitch_left, 0);
            qd_righ = dy->Cal_inverse_kinematic_Analytical(endposyawpitch_righ, 1);
            double kp[5] = {50, 50, 50, 30, 10};
            double kd[5] = {10, 10, 10, 10, 10};
            // if(d->time >10)
            // {
            //     kp[0] = 0;
            //     kp[1] = 0;
            //     kp[2] = 0;
            //     kp[3] = 0;
            //     kp[4] = 0;
            // }
            cout << "err: ";
            for (int i = 0; i < 5; i++)
            {
                cout << qd_left(i) - d->qpos[i] << " ";
                d->ctrl[i] = kp[i] * (qd_left(i) - d->qpos[i]);
                d->ctrl[i + 5] = kp[i] * (qd_righ(i) - d->qpos[5 + i]);
            }
            cout << endl;

            // cout << "qd_righ: "
            //      << qd_righ.transpose() << endl;
            // cout << "posd: " << endl
            //      << Te << endl;

            /* --------------------------*********TEST CODE END***********----------------------------*/
            dy->set_isUpdated();
            mj_step(m, d);
        }
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render  更新场景并渲染
        opt.frame = mjFRAME_WORLD; // 显示轴线
        // cam.lookat[0] = d->qpos[3]; // 视角跟随物体的x轴
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f};\n", cam.azimuth, cam.elevation, cam.distance, cam.lookat[0], cam.lookat[1], cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteModel(m);
    mj_deleteData(d);
    // delete ctrlComp;

    return 0;
}