LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := ros-slamdunk-node
LOCAL_CATEGORY_PATH := ros_packages
LOCAL_DESCRIPTION := Parrot S.L.A.M.dunk main ROS node

LOCAL_LIBRARIES := \
	libkalamos-context \
	ros-indigo-export \
	ros-slamdunk-msgs \
	ros-slamdunk-nodelets

SLAMDUNK_ROS_SDK_BUILD_DIR := $(call local-get-build-dir)/../ros-slamdunk/devel

LOCAL_EXPAND_CUSTOM_VARIABLES := 1
LOCAL_CMAKE_CONFIGURE_ARGS = \
	-DCATKIN_DEVEL_PREFIX=$(SLAMDUNK_ROS_SDK_BUILD_DIR) \
	%{CUSTOM_ROS_INDIGO_CMAKE_CONFIGURE_ARGS}

ifeq ("$(TARGET_CPU)", "tegrak1")
LOCAL_CMAKE_CONFIGURE_ARGS += -DCMAKE_INSTALL_PREFIX=/opt/ros-slamdunk/
endif

LOCAL_COPY_FILES := \
	../README.md:opt/ros-slamdunk/share/slamdunk_node/

LOCAL_EXTRA_DEPENDENCIES := \
	cfg/SLAMDunkNode.cfg \
	src/cvutils.cpp \
	src/cvutils.hpp \
	src/main.cpp

include $(BUILD_CMAKE)
