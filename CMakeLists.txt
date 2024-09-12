cmake_minimum_required(VERSION 3.16)

project(
    QLDsim
    DESCRIPTION "Quadruped locomotion"
    HOMEPAGE_URL "https://github.com/alefram/QLDsim"
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_language(CXX)

# source and header directory
set(SOURCE_DIR src)
set(HEADER_DIR src/visualization)
set(HEADER_DIR src/model)
set(HEADER_DIR src/planner)

add_executable(qldsim 
    ${SOURCE_DIR}/main.cc
    ${SOURCE_DIR}/visualization/visualization.cc
    ${SOURCE_DIR}/model/model.cc
    ${SOURCE_DIR}/planner/planner.cc
)

#add mujoco library
set(MUJOCO_INCLUDE_DIR /home/alexis/dev/mujoco-3.1.3/include/)
set(MUJOCO_LIB_DIR /home/alexis/dev/mujoco-3.1.3/lib/)

message(STATUS "Mujoco library path ${MUJOCO_LIB_DIR}")

target_include_directories(qldsim PRIVATE "${MUJOCO_INCLUDE_DIR}")
target_link_directories(qldsim PRIVATE "${MUJOCO_LIB_DIR}")

# add GLFW library
find_package(glfw3 REQUIRED)

target_link_libraries(qldsim PRIVATE glfw mujoco)

