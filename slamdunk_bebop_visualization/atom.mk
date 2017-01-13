LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := ros-bebop-visualization
LOCAL_CATEGORY_PATH := ros_packages
LOCAL_DESCRIPTION := Bebop ROS visualization tools

LOCAL_LIBRARIES := \
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
	src/ARSDKPanel.cpp \
	src/ARSDKPanel.hpp

include $(BUILD_CMAKE)
