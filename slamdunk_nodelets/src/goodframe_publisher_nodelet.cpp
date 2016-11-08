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

#include "goodframe_publisher_nodelet.hpp"

namespace slamdunk_nodelets
{

void GoodframePublisherNodelet::onInit()
{
    m_nh = getNodeHandle();
    m_privateNh = getPrivateNodeHandle();

    ros::SubscriberStatusCallback connection_cb = boost::bind(&GoodframePublisherNodelet::imageConnectionCallback, this);
    m_imagePublisher = m_privateNh.advertise<sensor_msgs::Image>("image_out", 1,
        connection_cb, // connect_cb
        connection_cb, // disconnect_cb
        {}, // tracked_object
        true); // latch: publish last message on new subscriber connection
}

void GoodframePublisherNodelet::imageConnectionCallback()
{
    NODELET_INFO("%s: %i", __func__, m_imagePublisher.getNumSubscribers());

    if (m_imagePublisher.getNumSubscribers())
        initInput();
    else
        deinitInput();
}

void GoodframePublisherNodelet::initInput()
{
    if (m_imageSubscriber)
        return;

    m_imageSubscriber.reset(new message_filters::Subscriber<sensor_msgs::Image>(m_nh, "image_in", 10));
    m_qualitySubscriber.reset(new message_filters::Subscriber<slamdunk_msgs::QualityStamped>(m_nh, "quality_in", 10));
    m_synchronizer.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, slamdunk_msgs::QualityStamped>(
        *m_imageSubscriber, *m_qualitySubscriber, 0));
    m_synchronizer->registerCallback(boost::bind(&GoodframePublisherNodelet::dataCallback, this, _1, _2));
}

void GoodframePublisherNodelet::deinitInput()
{
    if (!m_imageSubscriber)
        return;

    m_synchronizer.reset();
    m_qualitySubscriber.reset();
    m_imageSubscriber.reset();

}

void GoodframePublisherNodelet::dataCallback(sensor_msgs::Image::ConstPtr const& image,
                                             slamdunk_msgs::QualityStamped::ConstPtr const& quality)
{
    if (quality->quality.value == slamdunk_msgs::Quality::GOOD)
        m_imagePublisher.publish(image);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_nodelets::GoodframePublisherNodelet, nodelet::Nodelet);
