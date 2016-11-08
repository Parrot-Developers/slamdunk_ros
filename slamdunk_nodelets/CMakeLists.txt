cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_nodelets)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  slamdunk_msgs
  std_msgs
  tf2_ros
)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_library(goodframe_publisher_nodelet MODULE src/goodframe_publisher_nodelet.cpp)
target_compile_options(goodframe_publisher_nodelet PUBLIC -std=c++11)
target_link_libraries(goodframe_publisher_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(goodframe_publisher_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(pcl_xyzrgb_record_nodelet MODULE src/pcl_xyzrgb_record_nodelet.cpp)
target_compile_options(pcl_xyzrgb_record_nodelet PUBLIC -std=c++11)
target_link_libraries(pcl_xyzrgb_record_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(pcl_xyzrgb_record_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(keyframe_publisher_nodelet MODULE src/keyframe_publisher_nodelet.cpp)
target_compile_options(keyframe_publisher_nodelet PUBLIC -std=c++11)
target_link_libraries(keyframe_publisher_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(keyframe_publisher_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(pcl_xyzrgb_concat_node src/pcl_xyzrgb_concat_node.cpp)
target_compile_options(pcl_xyzrgb_concat_node PUBLIC -std=c++11)
target_link_libraries(pcl_xyzrgb_concat_node
  ${catkin_LIBRARIES}
)
add_dependencies(pcl_xyzrgb_concat_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(pcl_xyzrgb_concat_nodelet MODULE src/pcl_xyzrgb_concat_nodelet.cpp)
target_compile_options(pcl_xyzrgb_concat_nodelet PUBLIC -std=c++11)
target_link_libraries(pcl_xyzrgb_concat_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(pcl_xyzrgb_concat_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

# Pose to path nodelet
add_library(pose_to_path_nodelet MODULE src/pose_to_path_nodelet.cpp)
target_compile_options(pose_to_path_nodelet PUBLIC -std=c++11)
target_link_libraries(pose_to_path_nodelet
  ${catkin_LIBRARIES})
add_dependencies(pose_to_path_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

install(TARGETS
  goodframe_publisher_nodelet
  keyframe_publisher_nodelet
  pcl_xyzrgb_concat_node
  pcl_xyzrgb_concat_nodelet
  pcl_xyzrgb_record_nodelet
  pose_to_path_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
