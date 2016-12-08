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

#include "ARSDKPanel.hpp"

#include <QHeaderView>
#include <QStandardItemModel>
#include <QString>
#include <QTreeView>
#include <QVBoxLayout>

namespace slamdunk_bebop_visualization
{
ARSDKPanel::ARSDKPanel(QWidget *parent) : rviz::Panel(parent)
{
    m_batteryStateChangedSub =
        m_node.subscribe("/BebopNodelet/batteryStateChanged", 1, &ARSDKPanel::batteryStateChangedCallback, this);

    createGUI();
}

void ARSDKPanel::createGUI()
{
    this->setMinimumHeight(100);
    QVBoxLayout *layout = new QVBoxLayout;

    m_treeInfo = new QTreeView();
    m_model = new QStandardItemModel(0, 2);

    m_itemBatteryState = new QStandardItem("Battery state");
    m_itemBatteryState->setEditable(false);
    m_batteryStateValue = new QStandardItem("Unknown");
    m_batteryStateValue->setEditable(false);
    m_model->appendRow(QList<QStandardItem *>() << m_itemBatteryState << m_batteryStateValue);

    m_treeInfo->header()->hide();
    m_treeInfo->setModel(m_model);
    m_treeInfo->resizeColumnToContents(0);

    layout->addWidget(m_treeInfo);

    setLayout(layout);
}

void ARSDKPanel::batteryStateChangedCallback(const std_msgs::Float32ConstPtr &msg)
{
    m_batteryStateValue->setText(QString::number(msg->data) + " %");
}

}  // end namespace slamdunk_bebop_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_bebop_visualization::ARSDKPanel, rviz::Panel)
