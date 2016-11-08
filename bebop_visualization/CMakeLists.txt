cmake_minimum_required(VERSION 2.8.12)
project(bebop_visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  std_srvs
)

catkin_package(CATKIN_DEPENDS roscpp rviz)

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)
option(UseQt5 "UseQt5" OFF)
if (UseQt5)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  set(QTVERSION ${Qt5Widgets_VERSION})
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()
add_definitions(-DQT_NO_KEYWORDS)

# ARSDK panel
add_library(arsdk_panel
  src/ARSDKPanel.cpp
  ${MOC_FILES_PANEL_PLUGIN}
)
target_compile_options(arsdk_panel PUBLIC -std=c++11)
target_link_libraries(arsdk_panel ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(arsdk_panel
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  arsdk_panel
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
