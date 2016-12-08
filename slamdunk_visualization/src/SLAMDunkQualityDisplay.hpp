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

#ifndef SLAMDUNK_QUALITYDISPLAY_HPP
#define SLAMDUNK_QUALITYDISPLAY_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/message_filter_display.h>
#include <slamdunk_msgs/QualityStamped.h>
#endif

class QLabel;
class QTreeView;
class QStandardItemModel;
class QStandardItem;

namespace slamdunk_visualization
{
class SLAMDunkQualityDisplay : public rviz::MessageFilterDisplay<slamdunk_msgs::QualityStamped>
{
    Q_OBJECT
  public:
    SLAMDunkQualityDisplay();
    virtual ~SLAMDunkQualityDisplay();

    virtual void onInitialize();
    virtual void onDisable();
    virtual void onEnable();

  protected:
    virtual void processMessage(const slamdunk_msgs::QualityStampedConstPtr &msg);
    void clear();

  protected:
    QTreeView *m_treeInfo = nullptr;
    QStandardItemModel *m_model = nullptr;

    QStandardItem *m_itemQuality = nullptr;
    QStandardItem *m_qualityValue = nullptr;
};
} // end namespace slamdunk_visualization

#endif  // SLAMDUNK_QUALITYDISPLAY_HPP
