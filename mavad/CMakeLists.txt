include_directories(include)

file(GLOB_RECURSE mavades  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

set(DONT_BUILD )
GET_PROPERTY(local-ns3-libs GLOBAL PROPERTY ns3-libs)

add_library(ros_linker SHARED src/ros_linker.cc)
add_library(planner_ns3_utils SHARED src/planner_ns3_utils.cc)
add_library(planner_config SHARED src/planner_config.cc)
add_library(planner_ns3 SHARED src/planner_ns3.cc)

target_link_libraries(ros_linker ${local-ns3-libs} ${ns3-contrib-libs} )
target_link_libraries(planner_ns3_utils ${local-ns3-libs} ${ns3-contrib-libs})
target_link_libraries(planner_config ${local-ns3-libs} ${ns3-contrib-libs})
target_link_libraries(planner_ns3 ${local-ns3-libs} ${ns3-contrib-libs} planner_config planner_ns3_utils)

add_executable(mavad_main src/mavad_main.cc)
target_link_libraries(mavad_main ros_linker planner_ns3_utils planner_config planner_ns3)


