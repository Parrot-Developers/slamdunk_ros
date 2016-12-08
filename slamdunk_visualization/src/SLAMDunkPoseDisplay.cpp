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

#include "SLAMDunkPoseDisplay.hpp"

#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <QHeaderView>
#include <QLabel>
#include <QStandardItemModel>
#include <QString>
#include <QTreeView>
#include <QVBoxLayout>

namespace slamdunk_visualization
{
SLAMDunkPoseDisplay::SLAMDunkPoseDisplay() : MFDClass()
{
}

SLAMDunkPoseDisplay::~SLAMDunkPoseDisplay()
{
    delete m_treeInfo;
}

void SLAMDunkPoseDisplay::onInitialize()
{
    MFDClass::onInitialize();

    m_treeInfo = new QTreeView();
    m_model = new QStandardItemModel(0, 2);

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

    setAssociatedWidget(m_treeInfo);
}

void SLAMDunkPoseDisplay::onDisable()
{
    MFDClass::onDisable();
    clear();
}

void SLAMDunkPoseDisplay::onEnable()
{
    MFDClass::onEnable();
    clear();
}

void SLAMDunkPoseDisplay::processMessage(const geometry_msgs::PoseStampedConstPtr &msg)
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

void SLAMDunkPoseDisplay::clear()
{
    m_allPositionValues->setText("0; 0; 0");
    m_valuePositionX->setText("0");
    m_valuePositionY->setText("0");
    m_valuePositionZ->setText("0");
    m_allOrientationValues->setText("0; 0; 0");
    m_valueOrientationYaw->setText("0");
    m_valueOrientationPitch->setText("0");
    m_valueOrientationRoll->setText("0");
}

}  // end namespace slamdunk_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_visualization::SLAMDunkPoseDisplay, rviz::Display)
