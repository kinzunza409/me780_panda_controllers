cmake_minimum_required(VERSION 3.15)
project(franka_ik)

set(CMAKE_CXX_STANDARD 14)

# Find pybind11 from the system package location
# pybind11-dev installs cmake files here on Ubuntu 22.04
find_package(pybind11 REQUIRED PATHS /usr/share/cmake/pybind11)
find_package(Eigen3 REQUIRED)

pybind11_add_module(franka_ik pybind_wrapper.cpp)
target_link_libraries(franka_ik PRIVATE Eigen3::Eigen)