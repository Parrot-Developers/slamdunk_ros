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

#include "SLAMDunkPanel.hpp"

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <slamdunk_msgs/ServiceTrigger.h>
#include <std_srvs/Empty.h>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

namespace
{
void setServiceTrigger(const std::string &service, int trigger)
{
    dynamic_reconfigure::ReconfigureRequest request;

    dynamic_reconfigure::IntParameter param;
    param.name = service;
    param.value = trigger;
    request.config.ints.push_back(param);

    dynamic_reconfigure::ReconfigureResponse response;
    ros::service::call("/slamdunk_node/set_parameters", request, response);
}
}  // unnamed namespace

namespace slamdunk_visualization
{
SLAMDunkPanel::SLAMDunkPanel(QWidget *parent) : rviz::Panel(parent), m_nodeletLoader(false)
{
    createGUI();

    connect(m_buttonRestartSLAM, SIGNAL(clicked()), this, SLOT(restartSLAM()));
    connect(m_buttonRestartCapture, SIGNAL(clicked()), this, SLOT(restartCapture()));
    connect(m_buttonPclXyzrgbConcatClear, SIGNAL(clicked()), this, SLOT(pclXyzrgbConcatClear()));
    connect(m_buttonStartStreaming, SIGNAL(clicked()), this, SLOT(startStreaming()));
    connect(m_buttonStopStreaming, SIGNAL(clicked()), this, SLOT(stopStreaming()));
    connect(m_buttonRestartStreaming, SIGNAL(clicked()), this, SLOT(restartStreaming()));
    connect(m_buttonStartStreamingReception, SIGNAL(clicked()), this, SLOT(startStreamingReception()));
    connect(m_buttonStopStreamingReception, SIGNAL(clicked()), this, SLOT(stopStreamingReception()));
}

void SLAMDunkPanel::restartSLAM()
{
    callEmptyService("restart_slam");
}

void SLAMDunkPanel::restartCapture()
{
    callEmptyService("restart_capture");
}

void SLAMDunkPanel::pclXyzrgbConcatClear()
{
    callEmptyService("pcl_xyzrgb_concat_clear");
    callEmptyService("pcl_xyzrgb_concat_rviz_clear");
}

void SLAMDunkPanel::startStreaming()
{
    setServiceTrigger("service_trigger_streaming", slamdunk_msgs::ServiceTrigger::ALWAYS);
}

void SLAMDunkPanel::stopStreaming()
{
    setServiceTrigger("service_trigger_streaming", slamdunk_msgs::ServiceTrigger::ON_DEMAND);
}

void SLAMDunkPanel::restartStreaming()
{
    callEmptyService("restart_streaming");
}

void SLAMDunkPanel::startStreamingReception()
{
    ROS_INFO("%s", __PRETTY_FUNCTION__);
    auto loadedNodelets = m_nodeletLoader.listLoadedNodelets();

    std::string cams[] = {"cam0", "cam1"};
    // TODO get correct ports
    int ports[] = {5000, 5100};

    for (unsigned i = 0; i < std::extent<decltype(cams)>::value; ++i)
    {
        std::string cam = cams[i];
        int port = ports[i];
        std::string nodelet_name = cam + "_gscam_nodelet";

        if (std::find(loadedNodelets.begin(), loadedNodelets.end(), nodelet_name) != std::end(loadedNodelets))
            continue;

        nodelet::M_string remappings(ros::names::getRemappings());
        nodelet::V_string nargv;

        remappings["camera/image_raw"] = ros::names::resolve(nodelet_name + "/image_raw");
        remappings["camera/camera_info"] = ros::names::resolve(nodelet_name + "/camera_info");
        remappings["set_camera_info"] = ros::names::resolve(nodelet_name + "/set_camera_info");

        ros::param::set(nodelet_name + "/frame_id", cam);
        ros::param::set(nodelet_name + "/sync_sink", false);

        std::stringstream pipelineSs;
        pipelineSs << "udpsrc port=" << port << " caps=application/x-rtp ! rtph264depay ! avdec_h264 ! "
                                                "video/x-raw,format=I420 ! videoconvert";
        ros::param::set(nodelet_name + "/gscam_config", pipelineSs.str());

        m_nodeletLoader.load(nodelet_name, "gscam/GSCamNodelet", remappings, nargv);
    }
}

void SLAMDunkPanel::stopStreamingReception()
{
    ROS_INFO("%s", __PRETTY_FUNCTION__);
    auto loadedNodelets = m_nodeletLoader.listLoadedNodelets();
    std::string cams[] = {"cam0", "cam1"};
    for (std::string cam : cams)
    {
        std::string nodelet_name = cam + "_gscam_nodelet";
        m_nodeletLoader.unload(nodelet_name);
    }
}

void SLAMDunkPanel::callEmptyService(const std::string& serviceName)
{
    ros::ServiceClient client = m_node.serviceClient<std_srvs::Empty>(serviceName);
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("%s OK", serviceName.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service %s", serviceName.c_str());
    }
}

void SLAMDunkPanel::createGUI()
{
    this->setMinimumHeight(200);
    QVBoxLayout *layout = new QVBoxLayout;
    m_buttonRestartSLAM = new QPushButton("Restart SLAM");
    layout->addWidget(m_buttonRestartSLAM);

    m_buttonRestartCapture = new QPushButton("Restart capture");
    layout->addWidget(m_buttonRestartCapture);

    m_buttonPclXyzrgbConcatClear = new QPushButton("Clear PCL concat");
    layout->addWidget(m_buttonPclXyzrgbConcatClear);

    m_buttonStartStreaming = new QPushButton("Start Streaming");
    layout->addWidget(m_buttonStartStreaming);

    m_buttonStopStreaming = new QPushButton("Stop Streaming");
    layout->addWidget(m_buttonStopStreaming);

    m_buttonRestartStreaming = new QPushButton("Restart Streaming");
    layout->addWidget(m_buttonRestartStreaming);

    m_buttonStartStreamingReception = new QPushButton("Start Streaming Reception");
    layout->addWidget(m_buttonStartStreamingReception);

    m_buttonStopStreamingReception = new QPushButton("Stop Streaming Reception");
    layout->addWidget(m_buttonStopStreamingReception);

    setLayout(layout);
}

}  // end namespace slamdunk_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_visualization::SLAMDunkPanel, rviz::Panel)
