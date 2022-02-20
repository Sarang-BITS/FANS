project(mavad)
cmake_minimum_required(VERSION 3.18)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
nav_msgs
sensor_msgs
geometry_msgs
std_msgs
planner_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)

file(GLOB_RECURSE mavades  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

set(DONT_BUILD )
set(libraries_to_link ${libwifi} ${libapplications} ${libolsr} ${libnetanim} ${libinternet} ${libflow-monitor} ${libcore} ${libmobility} ${libconfig-store} ${libstats} ${libcsma} ${libbridge})

add_library(ros_linker        SHARED src/ros_linker.cc)
add_library(planner_ns3_utils SHARED src/planner_ns3_utils.cc)
add_library(planner_config    SHARED src/planner_config.cc)
add_library(planner_ns3       SHARED src/planner_ns3.cc)

target_link_libraries(ros_linker        ${catkin_LIBRARIES} ${ns3-libs}         ${ns3-contrib-libs})
target_link_libraries(planner_ns3_utils ${ns3-libs}         ${ns3-contrib-libs})
target_link_libraries(planner_config    ${ns3-libs}         ${ns3-contrib-libs})
target_link_libraries(planner_ns3       ${catkin_LIBRARIES} ${ns3-libs} "${libraries_to_link}" ${ns3-contrib-libs}   planner_config planner_ns3_utils)

add_executable(mavad_main src/mavad_main.cc)
target_link_libraries(mavad_main ${catkin_LIBRARIES} ros_linker planner_ns3_utils planner_config planner_ns3)

