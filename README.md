# Parrot S.L.A.M.dunk ROS Integration

This repository contains the ROS packages already bundled
on the [Parrot S.L.A.M.dunk](https://www.parrot.com/us/business-solutions/parrot-slamdunk).

It can be useful for advanced users who want to understand how the bundled packages work,
to modify them or to write a custom subscriber to one of the custom ROS messages.

* ROS version: Indigo
* Host PC recommended OS: Ubuntu 14.04
* Documentation: http://developer.parrot.com/docs/slamdunk/


## How to build the node yourself?

### Prerequisites

    sudo apt-get update
    sudo apt-get install build-essential git

ROS indigo:

- http://wiki.ros.org/indigo/Installation

### Prepare Workspace

    source /opt/ros/indigo/setup.bash
    mkdir -p ~/slamdunk_catkin_ws/src
    cd ~/slamdunk_catkin_ws/src
    catkin_init_workspace
    cd ~/slamdunk_catkin_ws/
    catkin_make -j2
    source devel/setup.bash

### Get slamdunk packages sources

    git clone https://github.com/Parrot-Developers/slamdunk_ros.git src/slamdunk_ros
    git clone https://github.com/Parrot-Developers/gscam.git src/gscam

### Disable slamdunk_node unless you build on Parrot S.L.A.M.dunk

The `slamdunk_node` and `slamdunk_bebop_robot` depends on `kalamos-context`,
which is a S.L.A.M.dunk-specific library.
To build the `slamdunk_ros` package outside of the Parrot S.L.A.M.dunk,
they need to be disabled.

    touch src/slamdunk_ros/slamdunk_node/CATKIN_IGNORE
    touch src/slamdunk_ros/slamdunk_bebop_robot/CATKIN_IGNORE

### Install

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    catkin_make -j2

**Note:** If you attempt to build for ROS Kinetic,
refer to the troubleshooting section about Qt:

* [Troubleshooting: Visualization packages issues with Qt 4.x](#Visualization-packages-issues-with-Qt-4_x)


## Viewing with rqt and rviz

Make sure `rviz` and `rqt` are installed:

    sudo apt-get install ros-indigo-rviz ros-indigo-rqt ros-indigo-rqt-plot

If you are not on the Parrot S.L.A.M.dunk itself,
configure you environment, e.g:

    export ROS_MASTER_URI="http://192.168.45.1:11311"
    export ROS_HOSTNAME=$(hostname).local

A perspective that plot the gyrometers and accelerometers in rqt:

    rqt --perspective-file $(rospack find slamdunk_visualization)/rqt/slamdunk.perspective

To visualize cameras, depthmap, SLAM pose and much more, use rviz:

    rviz -d $(rospack find slamdunk_visualization)/rviz/slamdunk.rviz


## Troubleshooting

### OpenCV compilation issue

You may encounter this error,
for example after upgrading a ROS package that depends on OpenCV
such as `ros-indigo-cv-bridge`, `ros-indigo-image-geometry`
or `ros-indigo-image-pipeline`:

    $ catkin_make -j2
    ...
    make[2]: *** No rule to make target `/usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.8', needed by `/home/slamdunk/catkin_ws/devel/lib/...'.  Stop.
    ...

This error is due to OpenCV4Tegra version which is slightly different from the
OpenCV package provided by Ubuntu.
The differences are:
* version: 2.4.8 => 2.4.13
* location: /usr/lib/arm-linux-gnueabihf/ => /usr/lib/
* libopencv_ocl.so.2.4.8: removed

To fix the issue, type:

    for file in share/cv_bridge/cmake/cv_bridgeConfig.cmake \
                share/image_proc/cmake/image_procConfig.cmake \
                share/image_geometry/cmake/image_geometryConfig.cmake \
                share/compressed_image_transport/cmake/compressed_image_transportConfig.cmake
    do
        sudo sed -e 's#;/usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.8##' \
            -e 's#/usr/lib/arm-linux-gnueabihf/libopencv_\([[:alnum:]]\+\)\.so\.2\.4\.8#/usr/lib/libopencv_\1.so.2.4.13#g' \
            -i /opt/ros/indigo/$file
    done

### Visualization packages issues with Qt 4.x

When building the packages against ROS Kinetic,
you may have a compilation issue:

    $ catkin_make -j2
    ...
    -- ==> add_subdirectory(slamdunk_ros/slamdunk_visualization)
    -- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
    qmake: could not exec '/usr/lib/x86_64-linux-gnu/qt4/bin/qmake': No such file or directory
    CMake Error at /usr/share/cmake-3.5/Modules/FindQt4.cmake:1326 (message):
      Found unsuitable Qt version "" from NOTFOUND, this code requires Qt 4.x
    Call Stack (most recent call first):
      slamdunk_ros/slamdunk_visualization/CMakeLists.txt:22 (find_package)


    -- Configuring incomplete, errors occurred!
    See also "/home/gpapin/slamdunk_catkin_ws/build/CMakeFiles/CMakeOutput.log".
    See also "/home/gpapin/slamdunk_catkin_ws/build/CMakeFiles/CMakeError.log".
    Invoking "cmake" failed
    $

Or less obviously,
encounter a segmentation fault
when running one of the `slamdunk_visualization` rviz plugins.

These errors happens because rviz uses Qt5 in ROS Kinetic,
instead of Qt4 in previous versions.

To fix these issues, turn on the `UseQt5` option:

    catkin_make -DUseQt5=ON -j2
