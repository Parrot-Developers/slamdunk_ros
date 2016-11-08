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

#ifndef BEBOP_NODELET_HPP
#define BEBOP_NODELET_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Empty.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include "kalamos_context.hpp"

namespace bebop_robot
{

class BebopNodelet : public nodelet::Nodelet
{
    void onInit() override;

    ros::NodeHandle m_privateNh;

    ros::Subscriber m_initSub;
    ros::Subscriber m_startSub;
    ros::Subscriber m_stopSub;
    ros::Subscriber m_emergencySub;
    ros::Subscriber m_pilotingVelocitySub;
    ros::Subscriber m_setMaxPitchRollRotationSpeedSub;
    ros::Subscriber m_setMaxRotationSpeedSub;
    ros::Subscriber m_setMaxVerticalSpeedSub;
    ros::Subscriber m_setMaxTiltSub;

    void initCb(const std_msgs::EmptyConstPtr &msg);
    void startCb(const std_msgs::EmptyConstPtr &msg);
    void stopCb(const std_msgs::EmptyConstPtr &msg);
    void emergencyCb(const std_msgs::EmptyConstPtr &msg);
    void pilotingVelocityCb(const geometry_msgs::TwistConstPtr &msg);
    void setMaxPitchRollRotationSpeedCB(const std_msgs::Float32ConstPtr &msg);
    void setMaxRotationSpeedCB(const std_msgs::Float32ConstPtr &msg);
    void setMaxVerticalSpeedCB(const std_msgs::Float32ConstPtr &msg);
    void setMaxTiltCB(const std_msgs::Float32ConstPtr &msg);

    ros::Publisher m_batteryStateChangedPub;

    void batteryStateChanged(float batteryState);

    std::unique_ptr<kalamos::Robot> m_robot;
};

} // namespace bebop_robot

#endif // BEBOP_NODELET_HPP
