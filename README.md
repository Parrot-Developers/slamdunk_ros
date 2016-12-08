# Parrot S.L.A.M.dunk ROS Integration

* ROS version: Indigo
* Documentation: http://developer.parrot.com/docs/slamdunk/

## How to build the node yourself?

### Prerequisites

    sudo apt-get update
    sudo apt-get install build-essential git

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


## Viewing with rqt and rviz

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

You may encounter this error, maybe after maybe after [re-]installing a ROS
package that depends on OpenCV such as `ros-indigo-cv-bridge`,
`ros-indigo-image-geometry` or `ros-indigo-image-pipeline`:

    $ catkin_make -j2
    ...
    make[2]: *** No rule to make target `/usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.8', needed by `/home/slamdunk/catkin_ws/devel/lib/...'.  Stop.
    ...

This error is due to OpenCV4Tegra version which is slightly different from the
OpenCV package provided by Ubuntu.
The differences are:
* version: 2.4.8 => 2.4.12
* location: /usr/lib/arm-linux-gnueabihf/ => /usr/lib/
* libopencv_ocl.so.2.4.8: removed

To fix the issue, type:

    for file in share/cv_bridge/cmake/cv_bridgeConfig.cmake \
                share/image_proc/cmake/image_procConfig.cmake \
                share/image_geometry/cmake/image_geometryConfig.cmake \
                share/compressed_image_transport/cmake/compressed_image_transportConfig.cmake
    do
        sudo sed -e 's#;/usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.8##' \
            -e 's#/usr/lib/arm-linux-gnueabihf/libopencv_\([[:alnum:]]\+\)\.so\.2\.4\.8#/usr/lib/libopencv_\1.so.2.4.12#g' \
            -i /opt/ros/indigo/$file
    done
