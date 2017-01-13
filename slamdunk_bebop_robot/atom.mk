LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := ros-bebop-robot
LOCAL_CATEGORY_PATH := ros_packages
LOCAL_DESCRIPTION := bebop robot node

LOCAL_LIBRARIES := \
	libkalamos-context \
	ros-indigo-export

SLAMDUNK_ROS_SDK_BUILD_DIR := $(call local-get-build-dir)/../ros-slamdunk/devel

LOCAL_EXPAND_CUSTOM_VARIABLES := 1
LOCAL_CMAKE_CONFIGURE_ARGS = \
	-DCATKIN_DEVEL_PREFIX=$(SLAMDUNK_ROS_SDK_BUILD_DIR) \
	%{CUSTOM_ROS_INDIGO_CMAKE_CONFIGURE_ARGS}

ifeq ("$(TARGET_CPU)", "tegrak1")
LOCAL_CMAKE_CONFIGURE_ARGS += -DCMAKE_INSTALL_PREFIX=/opt/ros-slamdunk/
endif

LOCAL_EXTRA_DEPENDENCIES := \
	launch/bebop_config.launch \
	launch/bebop_default.launch \
	launch/bebop_nodelet.launch \
	src/bebop_node.cpp \
	src/bebop_nodelet.hpp \
	src/bebop_nodelet.cpp

include $(BUILD_CMAKE)
