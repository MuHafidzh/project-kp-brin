cmake_minimum_required(VERSION 3.10)
project(TensorRTBert)

find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package(pybind11 REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${Python3_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

pybind11_add_module(tensorrtbert TensorRTBert.cpp)
target_link_libraries(tensorrtbert PRIVATE ${Python3_LIBRARIES} pybind11::module nvinfer ${CUDA_LIBRARIES})
set_target_properties(tensorrtbert PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}" SUFFIX "${PYTHON_MODULE_EXTENSION}")

# Determine the appropriate location to install the library
execute_process(
    COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
    OUTPUT_VARIABLE PYTHON_SITE_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Install rule
install(TARGETS tensorrtbert LIBRARY DESTINATION ${PYTHON_SITE_PATH})