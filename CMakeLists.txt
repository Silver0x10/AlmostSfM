cmake_minimum_required(VERSION 2.6)

project(sfm_1b)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#ds set internal includes (our files)
include_directories(src)

#ds add build targets
add_subdirectory(executables)