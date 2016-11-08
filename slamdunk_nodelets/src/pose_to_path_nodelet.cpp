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

#include "pose_to_path_nodelet.hpp"

#include "nav_msgs/Path.h"

namespace slamdunk_nodelets
{

void PoseToPathNodelet::onInit()
{
    m_nh = getNodeHandle();
    m_privateNh = getPrivateNodeHandle();

    ros::SubscriberStatusCallback pathKeyFrameStatusCb = boost::bind(&PoseToPathNodelet::pathKeyFramePublisherCb, this);
    m_pathKeyFramePub =
        m_privateNh.advertise<nav_msgs::Path>("path_keyframe", 1, pathKeyFrameStatusCb, pathKeyFrameStatusCb);

    ros::SubscriberStatusCallback pathStatusCb = boost::bind(&PoseToPathNodelet::pathPublisherCb, this);
    m_pathPub = m_privateNh.advertise<nav_msgs::Path>("path", 1, pathStatusCb, pathStatusCb);
}

void PoseToPathNodelet::pathKeyFramePublisherCb()
{
    NODELET_INFO("%s: %i", __func__, m_pathKeyFramePub.getNumSubscribers());

    if (m_pathKeyFramePub.getNumSubscribers())
    {
        if (m_mf_isKeyframeSub)
            return;

        m_mf_isKeyframeSub.reset(new message_filters::Subscriber<slamdunk_msgs::BoolStamped>(m_nh, "/keyframe", 1));
        m_mf_poseSub.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(m_nh, "/pose", 1));

        m_keyframeAndPoseSynchronizer.reset(
            new message_filters::TimeSynchronizer<slamdunk_msgs::BoolStamped, geometry_msgs::PoseStamped>(
                *m_mf_isKeyframeSub, *m_mf_poseSub, 1));
        m_keyframeAndPoseSynchronizer->registerCallback(
            boost::bind(&PoseToPathNodelet::onNewIsKeyframeAndPose, this, _1, _2));
    }
    else
    {
        m_keyframeAndPoseSynchronizer.reset();
        m_mf_isKeyframeSub.reset();
        m_mf_poseSub.reset();
        m_posesKeyframe.clear();
    }
}

void PoseToPathNodelet::pathPublisherCb()
{
    NODELET_INFO("%s: %i", __func__, m_pathPub.getNumSubscribers());

    if (m_pathPub.getNumSubscribers())
    {
        m_poseSub = m_privateNh.subscribe("/pose", 1, &PoseToPathNodelet::onNewPose, this);
    }
    else
    {
        m_poseSub.shutdown();
        m_poses.clear();
    }
}

void PoseToPathNodelet::onNewIsKeyframeAndPose(const slamdunk_msgs::BoolStampedConstPtr &keyframeMsg,
                                               const geometry_msgs::PoseStampedConstPtr &poseMsg)
{
    // This topic is notified of disconnection only when this topic publish something that's why we publish the path
    // even if no new pose is added.
    if (keyframeMsg->value)
    {
        m_posesKeyframe.push_back(*poseMsg);
    }
    nav_msgs::Path pathMsg;
    pathMsg.header = poseMsg->header;
    pathMsg.poses = m_posesKeyframe;
    m_pathKeyFramePub.publish(pathMsg);
}

void PoseToPathNodelet::onNewPose(const geometry_msgs::PoseStampedConstPtr &poseMsg)
{
    m_poses.push_back(*poseMsg);
    nav_msgs::Path pathMsg;
    pathMsg.header = poseMsg->header;
    pathMsg.poses = m_poses;
    m_pathPub.publish(pathMsg);
}

}  // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_nodelets::PoseToPathNodelet, nodelet::Nodelet);
