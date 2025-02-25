cmake_minimum_required(VERSION 3.6)
project(gpio)

add_compile_options(-std=c++14)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gpio_node src/gpio.cpp)

target_link_libraries(gpio_node
  ${catkin_LIBRARIES}
)

add_dependencies(gpio_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})