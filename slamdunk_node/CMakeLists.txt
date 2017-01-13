cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_node)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  interactive_markers
  nodelet
  octomap_msgs
  pcl_ros
  roscpp
  rospack # workaround https://github.com/ros/ros/issues/128
  sensor_msgs
  slamdunk_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/SLAMDunkNode.cfg
)

add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

catkin_package(CATKIN_DEPENDS roscpp)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_search_module(KALAMOS_CONTEXT REQUIRED kalamos-context)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(slamdunk_node src/main.cpp src/cvutils.cpp)
target_compile_options(slamdunk_node PUBLIC -std=c++11)
target_include_directories(slamdunk_node
  PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${KALAMOS_CONTEXT_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(slamdunk_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${KALAMOS_CONTEXT_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(slamdunk_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

install(TARGETS
        slamdunk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  cfg/SLAMDunkNode.cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
