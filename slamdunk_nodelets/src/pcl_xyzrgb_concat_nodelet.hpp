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

#ifndef __PCLXYZRGBCONCATNODELET_HPP__
#define __PCLXYZRGBCONCATNODELET_HPP__

#include <slamdunk_msgs/Save.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <nodelet/nodelet.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

namespace slamdunk_nodelets
{

class PclXYZRGBConcatNodelet : public nodelet::Nodelet
{
    void onInit() override;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_privateNh;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_pointCloudSubscriber;
    std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> m_tfPointCloudSubscriber;
    tf::TransformListener m_tfListener;

    ros::Publisher m_pointCloudPublisher;

    // Clear service
    ros::ServiceServer m_clearService;
    typedef std_srvs::Empty ClearService;
    bool clearServiceCb(ClearService::Request& req, ClearService::Response& resp);

    // Start service
    ros::ServiceServer m_startService;
    typedef std_srvs::Empty StartService;
    bool startServiceCb(StartService::Request& req, StartService::Response& resp);

    // Stop service
    ros::ServiceServer m_stopService;
    typedef std_srvs::Empty StopService;
    bool stopServiceCb(StopService::Request& req, StopService::Response& resp);

    // Save service
    ros::ServiceServer m_saveService;
    typedef slamdunk_msgs::Save SaveService;
    bool saveServiceCb(SaveService::Request& req, SaveService::Response& resp);

    bool m_started = false;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> m_pointCloud;

    void pointCloudConnectionCallback();
    void updateInput();
    void initInput();
    void deinitInput();

    void cloudCallback(sensor_msgs::PointCloud2::ConstPtr const& pcl);

    void insertScan(const tf::Point& sensorOrigin, pcl::PointCloud<pcl::PointXYZRGB> const& pc);

    void publish(const ros::Time& stamp);
};

}

#endif
