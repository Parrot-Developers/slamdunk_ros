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

#include "pcl_xyzrgb_record_nodelet.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

namespace slamdunk_nodelets
{
void PclXYZRGBRecordNodelet::onInit()
{
    m_nh = getNodeHandle();
    m_privateNh = getPrivateNodeHandle();

    m_startService = m_privateNh.advertiseService<StartService::Request, StartService::Response>(
        "start", boost::bind(&PclXYZRGBRecordNodelet::startServiceCb, this, _1, _2));
    m_stopService = m_privateNh.advertiseService<StopService::Request, StopService::Response>(
        "stop", boost::bind(&PclXYZRGBRecordNodelet::stopServiceCb, this, _1, _2));
}

bool PclXYZRGBRecordNodelet::startServiceCb(StartService::Request& req, StartService::Response& resp)
{
    NODELET_INFO("Start PCL XYZRGB record");

    if (m_pointCloudSubscriber)
        return false;

    if (!req.allowOverride && boost::filesystem::exists(req.path))
        return false;

    if (!boost::filesystem::create_directories(req.path))
        return false;

    m_path = req.path;
    m_pointCloudSubscriber.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "pcl_xyzrgb_in", 10));
    m_tfPointCloudSubscriber.reset(
        new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSubscriber, m_tfListener, "map", 10));
    m_tfPointCloudSubscriber->registerCallback(boost::bind(&PclXYZRGBRecordNodelet::cloudCallback, this, _1));
}

bool PclXYZRGBRecordNodelet::stopServiceCb(StopService::Request& req, StopService::Response& resp)
{
    NODELET_INFO("Stop PCL XYZRGB record");

    if (!m_pointCloudSubscriber)
        return false;

    m_tfPointCloudSubscriber.reset();
    m_pointCloudSubscriber.reset();
    m_currentNumber = 0;
    return true;
}

void PclXYZRGBRecordNodelet::cloudCallback(sensor_msgs::PointCloud2ConstPtr const& pclMessage)
{
    tf::StampedTransform sensorToWorldTransform;
    try
    {
        m_tfListener.lookupTransform("map", pclMessage->header.frame_id, pclMessage->header.stamp,
                                     sensorToWorldTransform);
    }
    catch (tf::TransformException& e)
    {
        NODELET_ERROR_STREAM("Transform error: " << e.what());
        return;
    }

    Eigen::Matrix4f sensorToWorldMatrix;
    pcl_ros::transformAsMatrix(sensorToWorldTransform, sensorToWorldMatrix);

    std::string path = (boost::format("%s/%04i.pcd") % m_path % (m_currentNumber++)).str();
    pcl::io::savePCDFile(path, *pclMessage, sensorToWorldMatrix.rightCols<1>(),
                         Eigen::Quaternionf(sensorToWorldMatrix.block<3, 3>(0, 0)), true);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_nodelets::PclXYZRGBRecordNodelet, nodelet::Nodelet);
