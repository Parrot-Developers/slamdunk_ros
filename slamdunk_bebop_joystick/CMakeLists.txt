cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_bebop_joystick)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
    config/logitech_F310.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
