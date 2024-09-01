#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

void mouse_button(GLFWwindow* window, int button, int act, int mods);

void mouse_move(GLFWwindow* window, double xpos, double ypos);

void scroll(GLFWwindow* window, double xoffset, double yoffset);

void initVisualization();

void updateAndRenderScene(GLFWwindow * window);

