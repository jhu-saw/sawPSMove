/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Aravind S Kumar, Anton Deguet
  Created on: 2025-08-19

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#pragma once

#ifndef _mtsPSMoveQtWidget_h
#define _mtsPSMoveQtWidget_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <QWidget>

class QLabel;
class QDoubleSpinBox;
class QPushButton;

#include <sawPSMove/sawPSMoveQtExport.h>

class SAW_PSMOVE_QT_EXPORT mtsPSMoveQtWidget : public QWidget, public mtsComponent
{
    Q_OBJECT
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR)

public:
    mtsPSMoveQtWidget(const std::string &name, QWidget *parent = nullptr);
    ~mtsPSMoveQtWidget() override;

    void Configure(const std::string &) override {}
    void Startup(void) override;
    void Cleanup(void) override;

private Q_SLOTS:
    void OnTimer();
    void OnSetLED();
    void OnRumbleChanged(double v);
    void OnResetOrientation();

private:
    // Required interface
	struct {
        mtsFunctionRead get_period_statistics;
        mtsFunctionRead measured_cp;
        mtsFunctionRead get_buttons;
        mtsFunctionRead get_trigger;
        mtsFunctionRead get_battery;
        mtsFunctionWrite set_LED;
        mtsFunctionWrite set_rumble;
        mtsFunctionVoid  reset_orientation;
	} device;

    // UI
    QTimer *Timer{nullptr};
    QLabel *PoseQ{nullptr};
    QLabel *Buttons{nullptr};
    QLabel *Trigger{nullptr};
    QLabel *Battery{nullptr};
    QDoubleSpinBox *Rumble{nullptr};
    QDoubleSpinBox *LED_R{nullptr};
    QDoubleSpinBox *LED_G{nullptr};
    QDoubleSpinBox *LED_B{nullptr};
    QPushButton *LED_Set{nullptr};
    QPushButton *ResetOri{nullptr};
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMoveQtWidget);

#endif // _mtsPSMoveQtWidget_h
