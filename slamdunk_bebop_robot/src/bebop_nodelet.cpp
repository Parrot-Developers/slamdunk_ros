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

#include "bebop_nodelet.hpp"

#include <geometry_msgs/Vector3.h>

namespace slamdunk_bebop_robot
{
void BebopNodelet::onInit()
{
    ROS_INFO("BebopNodelet init");

    m_privateNh = getPrivateNodeHandle();

    std::string bebopIp;
    m_privateNh.param<std::string>("bebop_ip", bebopIp, "192.168.43.1");
    ROS_INFO("Bebop IP is %s", bebopIp.c_str());

    m_robot = kalamos::createBebop(bebopIp);

    m_robot->setBatteryStateChangedCallback(std::bind(&BebopNodelet::batteryStateChanged, this, std::placeholders::_1));
    m_batteryStateChangedPub = m_privateNh.advertise<std_msgs::Float32>("batteryStateChanged", 1, true);
    // Publish one message to ensure information availability with latch mechanism
    std_msgs::Float32 batteryStateMsg;
    batteryStateMsg.data = m_robot->getBatteryState();
    m_batteryStateChangedPub.publish(batteryStateMsg);

    m_robot->setAttitudeChangedCallback(std::bind(&BebopNodelet::attitudeChanged, this, std::placeholders::_1));
    m_attitudeChangedPub = m_privateNh.advertise<geometry_msgs::Vector3>("attitudeChanged", 1, true);

    m_robot->setSpeedChangedCallback(std::bind(&BebopNodelet::speedChanged, this, std::placeholders::_1));
    m_speedChangedPub = m_privateNh.advertise<geometry_msgs::Vector3>("speedChanged", 1, true);

    m_initSub = m_privateNh.subscribe("init", 1, &BebopNodelet::initCb, this);
    m_startSub = m_privateNh.subscribe("start", 1, &BebopNodelet::startCb, this);
    m_stopSub = m_privateNh.subscribe("stop", 1, &BebopNodelet::stopCb, this);
    m_emergencySub = m_privateNh.subscribe("emergency", 1, &BebopNodelet::emergencyCb, this);
    m_pilotingVelocitySub = m_privateNh.subscribe("pilotingVelocity", 1, &BebopNodelet::pilotingVelocityCb, this);
    m_setMaxPitchRollRotationSpeedSub =
        m_privateNh.subscribe("setMaxPitchRollRotationSpeed", 1, &BebopNodelet::setMaxPitchRollRotationSpeedCB, this);
    m_setMaxRotationSpeedSub = m_privateNh.subscribe("setMaxRotationSpeed", 1, &BebopNodelet::setMaxRotationSpeedCB, this);
    m_setMaxVerticalSpeedSub = m_privateNh.subscribe("setMaxVerticalSpeed", 1, &BebopNodelet::setMaxVerticalSpeedCB, this);
    m_setMaxTiltSub = m_privateNh.subscribe("setMaxTilt", 1, &BebopNodelet::setMaxTiltCB, this);
}

void BebopNodelet::initCb(const std_msgs::EmptyConstPtr &msg)
{
    (void)msg;
    ROS_INFO("Robot: init requested...");
    m_robot->init();
}

void BebopNodelet::startCb(const std_msgs::EmptyConstPtr &msg)
{
    (void)msg;
    ROS_INFO("Robot: start requested...");
    m_robot->start();
}

void BebopNodelet::stopCb(const std_msgs::EmptyConstPtr &msg)
{
    (void)msg;
    ROS_INFO("Robot: stop requested...");
    m_robot->stop();
}

void BebopNodelet::emergencyCb(const std_msgs::EmptyConstPtr &msg)
{
    (void)msg;
    ROS_INFO("Robot: emergency requested...");
    m_robot->emergency();
}

void BebopNodelet::pilotingVelocityCb(const geometry_msgs::TwistConstPtr &msg)
{
    int roll = static_cast<int>(msg->linear.y);
    int pitch = static_cast<int>(msg->linear.x);
    int gaz = static_cast<int>(msg->linear.z);
    int yaw = static_cast<int>(msg->angular.z);
    ROS_INFO("Robot: pcmd roll %d pitch %d yaw %d gaz %d", roll, pitch, yaw, gaz);
    unsigned flag = 0;
    if (roll != 0 || pitch != 0)
    {
        flag = 1;
    }
    m_robot->pcmd(flag, roll, pitch, yaw, gaz);
}

void BebopNodelet::setMaxPitchRollRotationSpeedCB(const std_msgs::Float32ConstPtr &msg)
{
    ROS_INFO("Robot: set max pitch roll rotation speed: %f", msg->data);
    m_robot->setMaxPitchRollRotationSpeed(msg->data);
}

void BebopNodelet::setMaxRotationSpeedCB(const std_msgs::Float32ConstPtr &msg)
{
    ROS_INFO("Robot: set max rotation speed: %f", msg->data);
    m_robot->setMaxRotationSpeed(msg->data);
}

void BebopNodelet::setMaxVerticalSpeedCB(const std_msgs::Float32ConstPtr &msg)
{
    ROS_INFO("Robot: set max vertical speed: %f", msg->data);
    m_robot->setMaxVerticalSpeed(msg->data);
}

void BebopNodelet::setMaxTiltCB(const std_msgs::Float32ConstPtr &msg)
{
    ROS_INFO("Robot: set max tilt: %f", msg->data);
    m_robot->setMaxTilt(msg->data);
}

void BebopNodelet::batteryStateChanged(float batteryState)
{
    std_msgs::Float32 batteryStateMsg;
    batteryStateMsg.data = batteryState;
    m_batteryStateChangedPub.publish(batteryStateMsg);
}

void BebopNodelet::attitudeChanged(std::array<float, 3> attitude)
{
    geometry_msgs::Vector3 attitudeMsg;
    attitudeMsg.x = attitude[0];
    attitudeMsg.y = attitude[1];
    attitudeMsg.z = attitude[2];

    m_attitudeChangedPub.publish(attitudeMsg);
}

void BebopNodelet::speedChanged(std::array<float, 3> speed)
{
    geometry_msgs::Vector3 speedMsg;
    speedMsg.x = speed[0];
    speedMsg.y = speed[1];
    speedMsg.z = speed[2];

    m_speedChangedPub.publish(speedMsg);
}

}  // namespace slamdunk_bebop_robot

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_bebop_robot::BebopNodelet, nodelet::Nodelet);
