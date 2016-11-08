/*
 * Copyright (c) 2016 Parrot S.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int ac, char** av)
{
    ros::init(ac, av, "pcl_xyzrgb_concat_node");

    ros::NodeHandle n;

    nodelet::Loader nodelet(n);

    {
        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        // keyframe_publisher expects the following topics:
        // - input: Image image_in
        remappings["pcl_xyzrgb_in"] = ros::names::resolve("pcl_xyzrgb_kf_good");
        remappings["pcl_xyzrgb_out"] = ros::names::resolve("pcl_xyzrgb_concat");
        remappings["clear"] = ros::names::resolve("pcl_xyzrgb_concat_clear");
        nodelet.load(ros::this_node::getName(), "slamdunk_nodelets/PclXYZRGBConcatNodelet", remappings, nargv);
    }

    ros::spin();
    return 0;
}
