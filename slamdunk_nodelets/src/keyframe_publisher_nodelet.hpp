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

#ifndef __KEYFRAME_PUBLISHER_NODELET_HPP__
#define __KEYFRAME_PUBLISHER_NODELET_HPP__

#include <image_transport/image_transport.h>
#include <slamdunk_msgs/BoolStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace slamdunk_nodelets
{

class KeyframePublisherNodelet : public nodelet::Nodelet
{
    void onInit() override;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_privateNh;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> m_imageSubscriber;
    std::unique_ptr<message_filters::Subscriber<slamdunk_msgs::BoolStamped>> m_keyframeSubscriber;
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, slamdunk_msgs::BoolStamped>> m_synchronizer;

    image_transport::Publisher m_imagePublisher;

    void imageConnectionCallback();
    void initInput();
    void deinitInput();

    void dataCallback(sensor_msgs::Image::ConstPtr const& image,
                      slamdunk_msgs::BoolStamped::ConstPtr const& keyframe);
};

}

#endif
