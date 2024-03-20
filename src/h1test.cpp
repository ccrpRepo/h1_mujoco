#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>

#include "rpdynamics/dynamics.h"
#include "interface/IOmujoco.h"

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
        mju_error_s("Load model error: %s", error);
    d = mj_makeData(m);
    Init_window();

    ioInter = new IOmujoco(d);
    ctrlPlat = CtrlPlatform::MUJOCO;

    h1Robot *h1 = new h1Robot();
    Dynamics *dy = new Dynamics(h1);

    while(running)
    {
        running = !glfwWindowShouldClose(window);
        mjtNum simstart = d->time;
        while ((d->time - simstart) < 1.0 / 60.0)
        {
            /* --------------------------*********TEST CODE BEGIN*********----------------------------*/
            // xyz
            d->qpos[0] = 0;
            d->qpos[1] = 0;
            d->qpos[2] = 0.98;
            // quat
            d->qpos[3] = 1;
            d->qpos[4] = 0;
            d->qpos[5] = 0;
            d->qpos[6] = 0;
            // left leg
            d->qpos[7] = 0.43; // -0.43 ~ 0.43
            d->qpos[8] = 0.43; // -0.43 ~ 0.43
            d->qpos[9] = -1.57; // -1.57 ~ 1.57
            d->qpos[10] = -0.26; // -0.26 ~ 2.05
            d->qpos[11] = -0.62; // -0.87 ~ 0.52
            // right leg
            d->qpos[12] = 0; // -0.43 ~ 0.43
            d->qpos[13] = 0; // -0.43 ~ 0.43
            d->qpos[14] = -0.4; // -1.57 ~ 1.57
            d->qpos[15] = 0.8;  // -0.26 ~ 2.05
            d->qpos[16] = -0.4; // -0.87 ~ 0.52
            // torso
            d->qpos[17] = 2.35; // -2.35 ~ 2.35
            // left arm
            d->qpos[18] = 2.87; // -2.87 ~ 2.87
            d->qpos[19] = 3.11; // -0.34 ~ 3.11
            d->qpos[20] = 4.45; // -1.3 ~ 4.45
            d->qpos[21] = 2.61; // -1.25 ~ 2.61
            // right arm
            d->qpos[22] = 0; // -2.87 ~ 2.87
            d->qpos[23] = 0; // -0.34 ~ 3.11
            d->qpos[24] = 0; // -1.3 ~ 4.45
            d->qpos[25] = 0; // -1.25 ~ 2.61

            /* --------------------------*********TEST CODE END***********----------------------------*/
            dy->set_isUpdate();
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