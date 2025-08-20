/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Dorothy Hu
  Created on: 2017-01-20

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#pragma once

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <QTimerEvent>

#include <sawPSMove/sawPSMoveQtExport.h>

class CISST_EXPORT mtsPSMoveQtWidget : public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_ALL);

public:
    mtsPSMoveQtWidget(const std::string & componentName,
                      double periodInSeconds = 50.0 * cmn_ms);
    ~mtsPSMoveQtWidget() override {}

    void Configure(const std::string & filename = "") override;
    void Startup(void) override;
    void Cleanup(void) override;

protected:
    void timerEvent(QTimerEvent * event) override;

private:
    void setupUi(void);
    void setPoseLabels(const prmPositionCartesianGet & cp);

    struct {
        mtsFunctionRead MeasuredCP;
        mtsFunctionRead GetPeriodStatistics;
        prmPositionCartesianGet CP;
    } Dev;

    // UI
    QLabel * lblPosX{nullptr};
    QLabel * lblPosY{nullptr};
    QLabel * lblPosZ{nullptr};
    QLabel * lblRoll{nullptr};
    QLabel * lblPitch{nullptr};
    QLabel * lblYaw{nullptr};
    mtsQtWidgetIntervalStatistics * statsWidget{nullptr};

    int TimerPeriodMs;
};
CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMoveQtWidget);
