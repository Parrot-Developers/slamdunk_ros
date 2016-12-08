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

#include "pcl_xyzrgb_concat_nodelet.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

namespace slamdunk_nodelets
{

void PclXYZRGBConcatNodelet::onInit()
{
    m_nh = getNodeHandle();
    m_privateNh = getPrivateNodeHandle();

    ros::SubscriberStatusCallback connection_cb = boost::bind(&PclXYZRGBConcatNodelet::pointCloudConnectionCallback, this);
    m_pointCloudPublisher = m_privateNh.advertise<sensor_msgs::PointCloud2>("pcl_xyzrgb_out", 1,
        connection_cb, // connect_cb
        connection_cb, // disconnect_cb
        {}, // tracked_object
        true); // latch: publish last message on new subscriber connection

    m_clearService = m_privateNh.advertiseService<ClearService::Request, ClearService::Response>("clear",
                                                 boost::bind(&PclXYZRGBConcatNodelet::clearServiceCb, this, _1, _2));
    m_rvizClearService = m_privateNh.advertiseService<RvizClearService::Request, RvizClearService::Response>("rviz_clear",
                                                 boost::bind(&PclXYZRGBConcatNodelet::rvizClearServiceCb, this, _1, _2));
    m_startService = m_privateNh.advertiseService<StartService::Request, StartService::Response>("start",
                                                 boost::bind(&PclXYZRGBConcatNodelet::startServiceCb, this, _1, _2));
    m_stopService = m_privateNh.advertiseService<StopService::Request, StopService::Response>("stop",
                                                 boost::bind(&PclXYZRGBConcatNodelet::stopServiceCb, this, _1, _2));
    m_saveService = m_privateNh.advertiseService<SaveService::Request, SaveService::Response>("save",
                                                 boost::bind(&PclXYZRGBConcatNodelet::saveServiceCb, this, _1, _2));
}

bool PclXYZRGBConcatNodelet::clearServiceCb(ClearService::Request& req, ClearService::Response& resp)
{
    NODELET_INFO("Clear PCL XYZRGB concat");
    m_pointCloud.reset();
    publish(ros::Time::now());
    return true;
}

bool PclXYZRGBConcatNodelet::rvizClearServiceCb(RvizClearService::Request& req, RvizClearService::Response& resp)
{
    NODELET_INFO("Clear rviz PCL XYZRGB concat");

    if (!m_emptyPointCloud)
    {
        m_emptyPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointXYZRGB point;
        point.x = 999999.0f;
        point.y = 999999.0f;
        point.z = 999999.0f;
        m_emptyPointCloud->push_back(point);
    }

    publish(ros::Time::now(), *m_emptyPointCloud);
    return true;
}

bool PclXYZRGBConcatNodelet::startServiceCb(StartService::Request& req, StartService::Response& resp)
{
    NODELET_INFO("Start PCL XYZRGB concat");
    m_started = true;
    updateInput();
    return true;
}

bool PclXYZRGBConcatNodelet::stopServiceCb(StopService::Request& req, StopService::Response& resp)
{
    NODELET_INFO("Stop PCL XYZRGB concat");
    m_started = false;
    updateInput();
    return true;
}

bool PclXYZRGBConcatNodelet::saveServiceCb(SaveService::Request& req, SaveService::Response& resp)
{
    NODELET_INFO("Save PCL XYZRGB concat");
    if (!m_pointCloud)
        return false;

    if (!req.allowOverride && boost::filesystem::exists(req.path))
        return false;

    pcl::io::savePCDFileBinary(req.path, *m_pointCloud);
    return true;
}

void PclXYZRGBConcatNodelet::pointCloudConnectionCallback()
{
    NODELET_INFO("%s: %i", __func__, m_pointCloudPublisher.getNumSubscribers());

    updateInput();
}

void PclXYZRGBConcatNodelet::updateInput()
{
    if (m_started || m_pointCloudPublisher.getNumSubscribers())
        initInput();
    else
        deinitInput();
}

void PclXYZRGBConcatNodelet::initInput()
{
    if (m_pointCloudSubscriber)
        return;

    m_pointCloudSubscriber.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "pcl_xyzrgb_in", 10));
    m_tfPointCloudSubscriber.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSubscriber, m_tfListener, "map", 10));
    m_tfPointCloudSubscriber->registerCallback(boost::bind(&PclXYZRGBConcatNodelet::cloudCallback, this, _1));

}

void PclXYZRGBConcatNodelet::deinitInput()
{
    if (!m_pointCloudSubscriber)
        return;

    m_tfPointCloudSubscriber.reset();
    m_pointCloudSubscriber.reset();
}

void PclXYZRGBConcatNodelet::cloudCallback(sensor_msgs::PointCloud2ConstPtr const& pclMessage)
{
    tf::StampedTransform sensorToWorldTransform;
    try
    {
        m_tfListener.lookupTransform("map", pclMessage->header.frame_id, pclMessage->header.stamp, sensorToWorldTransform);
    }
    catch (tf::TransformException& e)
    {
        NODELET_ERROR_STREAM("Transform error: " << e.what());
        return;
    }

    Eigen::Matrix4f sensorToWorldMatrix;
    pcl_ros::transformAsMatrix(sensorToWorldTransform, sensorToWorldMatrix);

    pcl::PointCloud<pcl::PointXYZRGB> pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*pclMessage, pc);

    pcl::transformPointCloud(pc, pc, sensorToWorldMatrix);

    insertScan(sensorToWorldTransform.getOrigin(), pc);

    if (m_pointCloudPublisher.getNumSubscribers())
        publish(pclMessage->header.stamp);
}

void PclXYZRGBConcatNodelet::insertScan(const tf::Point& sensorOrigin, pcl::PointCloud<pcl::PointXYZRGB> const& pc)
{
    if (!m_pointCloud || m_pointCloud == m_emptyPointCloud)
        m_pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (auto it = pc.begin(), ite = pc.end(); it != ite; ++it)
    {
        pcl::PointXYZRGB const& point = *it;
        if (std::isfinite(point.x))
        {
            m_pointCloud->push_back(point);
        }
    }

    // 2 filters to reduce the size of the concatenation
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setLeafSize(0.05, 0.05, 0.05);
        filter.setInputCloud(m_pointCloud);

        pcl::PointCloud<pcl::PointXYZRGB> filtered;
        filter.filter(filtered);
        *m_pointCloud = filtered;
    }
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> filter;
        filter.setRadiusSearch(0.05);
        filter.setMinNeighborsInRadius(2);
        filter.setInputCloud(m_pointCloud);

        pcl::PointCloud<pcl::PointXYZRGB> filtered;
        filter.filter(filtered);
        *m_pointCloud = filtered;
    }
}

void PclXYZRGBConcatNodelet::publish(const ros::Time& stamp)
{
    if (!m_pointCloud)
        return;
    publish(stamp, *m_pointCloud);
}

void PclXYZRGBConcatNodelet::publish(const ros::Time& stamp, const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud)
{
    sensor_msgs::PointCloud2Ptr pc(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(pointCloud, *pc);
    pc->header.frame_id = "map";
    pc->header.stamp = stamp;
    m_pointCloudPublisher.publish(pc);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_nodelets::PclXYZRGBConcatNodelet, nodelet::Nodelet);
