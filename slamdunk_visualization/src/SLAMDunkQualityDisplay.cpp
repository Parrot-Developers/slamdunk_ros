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

#include "SLAMDunkQualityDisplay.hpp"

#include <QHeaderView>
#include <QLabel>
#include <QStandardItemModel>
#include <QString>
#include <QTreeView>
#include <QVBoxLayout>

namespace slamdunk_visualization
{
SLAMDunkQualityDisplay::SLAMDunkQualityDisplay() : MFDClass()
{
}

SLAMDunkQualityDisplay::~SLAMDunkQualityDisplay()
{
    delete m_treeInfo;
}

void SLAMDunkQualityDisplay::onInitialize()
{
    MFDClass::onInitialize();

    m_treeInfo = new QTreeView();
    m_model = new QStandardItemModel(0, 2);

    m_itemQuality = new QStandardItem("SLAM quality");
    m_itemQuality->setEditable(false);
    m_qualityValue = new QStandardItem("Unknown");
    m_qualityValue->setEditable(false);
    m_model->appendRow(QList<QStandardItem *>() << m_itemQuality << m_qualityValue);

    m_treeInfo->header()->hide();
    m_treeInfo->setModel(m_model);
    m_treeInfo->resizeColumnToContents(0);
    m_treeInfo->setAnimated(true);

    setAssociatedWidget(m_treeInfo);
}

void SLAMDunkQualityDisplay::onDisable()
{
    MFDClass::onDisable();
    clear();
}

void SLAMDunkQualityDisplay::onEnable()
{
    MFDClass::onEnable();
    clear();
}


void SLAMDunkQualityDisplay::processMessage(const slamdunk_msgs::QualityStampedConstPtr &msg)
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

void SLAMDunkQualityDisplay::clear()
{
    m_qualityValue->setText("Unknown");
    QColor itemBackgroundColor = Qt::white;
    m_qualityValue->setBackground(QBrush(itemBackgroundColor));
    m_itemQuality->setBackground(QBrush(itemBackgroundColor));
}

}  // end namespace slamdunk_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slamdunk_visualization::SLAMDunkQualityDisplay, rviz::Display)
