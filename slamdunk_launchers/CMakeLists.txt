cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_launchers)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
