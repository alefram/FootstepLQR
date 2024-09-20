#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include "visualization/visualization.h"

mjvScene scn; //scene                      
mjvOption opt; // visualization options                     
mjvCamera cam; // camera                     
mjrContext con; // openGL context
mjData* data = nullptr; // mujoco data
mjModel* model = nullptr; // mujoco model

int main(int argc, char **argv) {
    model = mj_loadXML("anybotics_anymal_b/scene.xml", nullptr, nullptr, 0);
    data = mj_makeData(model);

    //init GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    initVisualization();

    // install GLFW mouse callbacks
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    while(!glfwWindowShouldClose(window)) {
        mjtNum simstart = data->time;
        while (data->time - simstart < 1.0/60.0) {
            mj_step(model, data);
        }

        // update and render scene
        updateAndRenderScene(window);

        // swap OpenGL buffers and handle events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
 
    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    //Free the memory used by the model and data
    mj_deleteModel(model);
    mj_deleteData(data);

    return 0;
}


