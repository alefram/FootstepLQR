cmake_minimum_required(VERSION 3.16)

project(
    QLDsim
    DESCRIPTION "Quadruped locomotion differentiable simulator"
    HOMEPAGE_URL "https://github.com/alefram/QLDsim"
)

enable_language(CXX)

add_executable(qldsim main.cc)

set(MUJOCO_INCLUDE_DIR /home/alexis/dev/mujoco-3.1.3/include/)
set(MUJOCO_LIB_DIR /home/alexis/dev/mujoco-3.1.3/lib/)

message(STATUS "Mujoco library path ${MUJOCO_LIB_DIR}")

target_include_directories(qldsim PUBLIC "${MUJOCO_INCLUDE_DIR}")
target_link_directories(qldsim PUBLIC "${MUJOCO_LIB_DIR}")
target_link_libraries(qldsim mujoco)
