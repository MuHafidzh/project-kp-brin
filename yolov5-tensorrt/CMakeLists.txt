cmake_minimum_required(VERSION 3.6)
project(yolov5-tensorrt)

add_compile_options(-std=c++14)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g -Wall -Wextra -Wno-deprecated -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g -pthread")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
if(NOT DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

option(BUILD_PYTHON "Build the Python bindings" OFF)


## Pkg-Config support when installing
include(GNUInstallDirs)
configure_file(yolov5-tensorrt.pc.in yolov5-tensorrt.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/yolov5-tensorrt.pc
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)


## Libraries
#####################

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)


if(BUILD_PYTHON)
    # use pybind11 to build the Python bindings
    find_package(pybind11 REQUIRED)
endif()


## Sources
#####################

include_directories(include)
file(GLOB_RECURSE YOLOV5_INCLUDE_FILES "include/*.hpp*")

add_subdirectory(src)

add_subdirectory(bindings)

## Examples
#####################

add_subdirectory(examples)



find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
