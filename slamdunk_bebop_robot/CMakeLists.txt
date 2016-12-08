cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_bebop_robot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy_teleop
  nodelet
  roscpp
  rospack # workaround https://github.com/ros/ros/issues/128
  std_msgs
  std_srvs
)

catkin_package()

find_package(PkgConfig REQUIRED)
pkg_search_module(KALAMOS_CONTEXT REQUIRED kalamos-context)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(bebop_node src/bebop_node.cpp)
target_compile_options(bebop_node PUBLIC -std=c++11)
target_link_libraries(bebop_node
  ${catkin_LIBRARIES}
)
add_dependencies(bebop_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(bebop_nodelet MODULE src/bebop_nodelet.cpp)
target_compile_options(bebop_nodelet PUBLIC -std=c++11)
target_include_directories(bebop_nodelet
  PRIVATE
  ${KALAMOS_CONTEXT_INCLUDE_DIRS})
target_link_libraries(bebop_nodelet
    ${KALAMOS_CONTEXT_LIBRARIES}
    ${catkin_LIBRARIES})
add_dependencies(bebop_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

install(TARGETS
        bebop_node
        bebop_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
