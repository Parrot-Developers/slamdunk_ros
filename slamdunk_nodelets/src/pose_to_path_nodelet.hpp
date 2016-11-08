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

#ifndef POSE_TO_PATH_NODELET_HPP
#define POSE_TO_PATH_NODELET_HPP

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <memory>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "slamdunk_msgs/BoolStamped.h"

namespace slamdunk_nodelets
{
/// This nodelet publish two topics:
/// - /path A list of all the poses
/// - /path_keyframe A list of the poses associated with a key frame
class PoseToPathNodelet : public nodelet::Nodelet
{
    void onInit() override;
    void pathKeyFramePublisherCb();
    void pathPublisherCb();

    ros::NodeHandle m_nh;
    ros::NodeHandle m_privateNh;

    std::unique_ptr<message_filters::Subscriber<slamdunk_msgs::BoolStamped>> m_mf_isKeyframeSub;
    std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> m_mf_poseSub;
    std::unique_ptr<message_filters::TimeSynchronizer<slamdunk_msgs::BoolStamped, geometry_msgs::PoseStamped>>
        m_keyframeAndPoseSynchronizer;

    ros::Subscriber m_poseSub;

    ros::Publisher m_pathKeyFramePub;
    ros::Publisher m_pathPub;

    void onNewIsKeyframeAndPose(const slamdunk_msgs::BoolStampedConstPtr &keyframeMsg,
                                const geometry_msgs::PoseStampedConstPtr &poseMsg);

    void onNewPose(const geometry_msgs::PoseStampedConstPtr &poseMsg);

    std::vector<geometry_msgs::PoseStamped> m_posesKeyframe;
    std::vector<geometry_msgs::PoseStamped> m_poses;
};
}

#endif  // POSE_TO_PATH_NODELET_HPP
