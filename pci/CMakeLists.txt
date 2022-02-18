cmake_minimum_required(VERSION 2.8.3)
project(pci)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})


cs_add_library(
  ${PROJECT_NAME}
  src/drone.cpp)

cs_add_executable(pci_node src/pci_node.cpp)
target_link_libraries(pci_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)
