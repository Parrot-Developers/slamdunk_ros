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

#include <angles/angles.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QStandardItemModel>
#include <QString>
#include <QTreeView>
#include <QVBoxLayout>

namespace slamdunk_visualization
{
SLAMDunkPanel::SLAMDunkPanel(QWidget *parent) : rviz::Panel(parent)
{
    m_poseSub = m_node.subscribe("pose", 1, &SLAMDunkPanel::poseCallback, this);
    m_qualitySub = m_node.subscribe("quality", 1, &SLAMDunkPanel::qualityCallback, this);

    createGUI();

    connect(m_buttonRestartSLAM, SIGNAL(clicked()), this, SLOT(restartSLAM()));
    connect(m_buttonRestartCapture, SIGNAL(clicked()), this, SLOT(restartCapture()));
    connect(m_buttonPclXyzrgbConcatClear, SIGNAL(clicked()), this, SLOT(pclXyzrgbConcatClear()));
}

void SLAMDunkPanel::restartSLAM()
{
    ros::ServiceClient client = m_node.serviceClient<std_srvs::Empty>("restart_slam");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("restart_slam OK");
    }
    else
    {
        ROS_ERROR("Failed to call service restart_slam");
    }
}

void SLAMDunkPanel::restartCapture()
{
    ros::ServiceClient client = m_node.serviceClient<std_srvs::Empty>("restart_capture");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("restart_capture OK");
    }
    else
    {
        ROS_ERROR("Failed to call service restart_capture");
    }
}

void SLAMDunkPanel::pclXyzrgbConcatClear()
{
    ros::ServiceClient client = m_node.serviceClient<std_srvs::Empty>("pcl_xyzrgb_concat_clear");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("pcl_xyzrgb_concat_clear OK");
    }
    else
    {
        ROS_ERROR("Failed to call service pcl_xyzrgb_concat_clear");
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

    m_treeInfo = new QTreeView();
    m_model = new QStandardItemModel(0, 2);

    m_itemQuality = new QStandardItem("SLAM quality");
    m_itemQuality->setEditable(false);
    m_qualityValue = new QStandardItem("Unknown");
    m_qualityValue->setEditable(false);
    m_model->appendRow(QList<QStandardItem *>() << m_itemQuality << m_qualityValue);

    QStandardItem *itemPosition = new QStandardItem("Position");
    itemPosition->setEditable(false);
    m_allPositionValues = new QStandardItem("0; 0; 0");
    m_model->appendRow(QList<QStandardItem *>() << itemPosition << m_allPositionValues);

    QStandardItem *namePositionX = new QStandardItem(QString("X"));
    namePositionX->setEditable(false);

    QStandardItem *namePositionY = new QStandardItem(QString("Y"));
    namePositionY->setEditable(false);

    QStandardItem *namePositionZ = new QStandardItem(QString("Z"));
    namePositionZ->setEditable(false);

    m_valuePositionX = new QStandardItem(QString("0"));
    m_valuePositionX->setEditable(false);

    m_valuePositionY = new QStandardItem(QString("0"));
    m_valuePositionY->setEditable(false);

    m_valuePositionZ = new QStandardItem(QString("0"));
    m_valuePositionZ->setEditable(false);

    itemPosition->appendRow(QList<QStandardItem *>() << namePositionX << m_valuePositionX);
    itemPosition->appendRow(QList<QStandardItem *>() << namePositionY << m_valuePositionY);
    itemPosition->appendRow(QList<QStandardItem *>() << namePositionZ << m_valuePositionZ);

    QStandardItem *itemOrientation = new QStandardItem("Orientation");
    itemOrientation->setEditable(false);
    m_allOrientationValues = new QStandardItem("0; 0; 0");
    m_model->appendRow(QList<QStandardItem *>() << itemOrientation << m_allOrientationValues);

    QStandardItem *nameOrientationYaw = new QStandardItem(QString("Yaw"));
    nameOrientationYaw->setEditable(false);

    QStandardItem *nameOrientationPitch = new QStandardItem(QString("Pitch"));
    nameOrientationPitch->setEditable(false);

    QStandardItem *nameOrientationRoll = new QStandardItem(QString("Roll"));
    nameOrientationRoll->setEditable(false);

    m_valueOrientationYaw = new QStandardItem(QString("0"));
    m_valueOrientationYaw->setEditable(false);

    m_valueOrientationPitch = new QStandardItem(QString("0"));
    m_valueOrientationPitch->setEditable(false);

    m_valueOrientationRoll = new QStandardItem(QString("0"));
    m_valueOrientationRoll->setEditable(false);

    itemOrientation->appendRow(QList<QStandardItem *>() << nameOrientationYaw << m_valueOrientationYaw);
    itemOrientation->appendRow(QList<QStandardItem *>() << nameOrientationPitch << m_valueOrientationPitch);
    itemOrientation->appendRow(QList<QStandardItem *>() << nameOrientationRoll << m_valueOrientationRoll);

    m_treeInfo->header()->hide();
    m_treeInfo->setModel(m_model);
    m_treeInfo->resizeColumnToContents(0);
    m_treeInfo->setAnimated(true);

    layout->addWidget(m_treeInfo);

    setLayout(layout);
}

void SLAMDunkPanel::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    const int precision = 2;
    QString positionX = QString::number(msg->pose.position.x, 'f', precision);
    QString positionY = QString::number(msg->pose.position.y, 'f', precision);
    QString positionZ = QString::number(msg->pose.position.z, 'f', precision);

    m_allPositionValues->setText(positionX + "; " + positionY + "; " + positionZ);

    m_valuePositionX->setText(positionX);
    m_valuePositionY->setText(positionY);
    m_valuePositionZ->setText(positionZ);

    tf::Quaternion quatOrientation(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                                   msg->pose.orientation.w);
    tf::Matrix3x3 matOrientation(quatOrientation);
    double roll, pitch, yaw;
    matOrientation.getRPY(roll, pitch, yaw);

    QString orientationYaw = QString::number(angles::to_degrees(yaw), 'f', precision);
    QString orientationPitch = QString::number(angles::to_degrees(pitch), 'f', precision);
    QString orientationRoll = QString::number(angles::to_degrees(roll), 'f', precision);

    m_allOrientationValues->setText(orientationYaw + "; " + orientationPitch + "; " + orientationRoll);

    m_valueOrientationYaw->setText(orientationYaw);
    m_valueOrientationPitch->setText(orientationPitch);
    m_valueOrientationRoll->setText(orientationRoll);
}

void SLAMDunkPanel::qualityCallback(const slamdunk_msgs::QualityStampedConstPtr &msg)
{
    QColor itemBackgroundColor = Qt::white;
    switch (msg->quality.value)
    {
        case slamdunk_msgs::Quality::UNKNOWN:
            m_qualityValue->setText("Unknown");
            itemBackgroundColor = Qt::white;
            break;
        case slamdunk_msgs::Quality::GOOD:
            m_qualityValue->setText("Good");
            itemBackgroundColor = Qt::green;
            break;
        case slamdunk_msgs::Quality::HAZARDOUS:
            m_qualityValue->setText("Hazardous");
            itemBackgroundColor = Qt::yellow;
            break;
        case slamdunk_msgs::Quality::BAD:
            m_qualityValue->setText("Bad");
            itemBackgroundColor = Qt::red;
            break;
        case slamdunk_msgs::Quality::LOST:
            m_qualityValue->setText("Lost");
            itemBackgroundColor = Qt::red;
            break;
    }
    m_qualityValue->setBackground(QBrush(itemBackgroundColor));
    m_itemQuality->setBackground(QBrush(itemBackgroundColor));
}

}  // end namespace slamdunk_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_visualization::SLAMDunkPanel, rviz::Panel)
