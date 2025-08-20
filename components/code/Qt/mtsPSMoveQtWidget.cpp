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


#include <sawPSMove/mtsPSMoveQtWidget.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstCommon/cmnLogger.h>
#include <cmath>
#include <QString>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPSMoveQtWidget, mtsComponent, std::string);

static inline void rotToRPY(const vctRot3 & R, double & roll, double & pitch, double & yaw)
{
    // XYZ convention
    roll  = std::atan2(R.Element(2,1), R.Element(2,2));
    pitch = std::asin( - R.Element(2,0) );
    yaw   = std::atan2(R.Element(1,0), R.Element(0,0));
}

mtsPSMoveQtWidget::mtsPSMoveQtWidget(const std::string & componentName,
                                     double periodInSeconds)
    : mtsComponent(componentName),
      TimerPeriodMs(static_cast<int>(periodInSeconds * 1000.0))
{
    mtsInterfaceRequired * req = AddInterfaceRequired("Controller");
    if (req) {
        req->AddFunction("measured_cp", Dev.MeasuredCP);
        req->AddFunction("GetPeriodStatistics", Dev.GetPeriodStatistics);
    }
    setupUi();
    startTimer(TimerPeriodMs);
}

void mtsPSMoveQtWidget::Configure(const std::string &)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure()" << std::endl;
}

void mtsPSMoveQtWidget::Startup(void)
{
    if (!parent()) {
        show();
    }
}

void mtsPSMoveQtWidget::Cleanup(void)
{
    this->hide();
}

void mtsPSMoveQtWidget::setupUi()
{
    auto * poseBox = new QGroupBox("PSMove Pose");
    auto * grid = new QGridLayout;

    lblPosX = new QLabel("0.000");
    lblPosY = new QLabel("0.000");
    lblPosZ = new QLabel("0.000");
    lblRoll = new QLabel("0.000");
    lblPitch= new QLabel("0.000");
    lblYaw  = new QLabel("0.000");

    int r=0;
    grid->addWidget(new QLabel("X (m):"), r, 0); grid->addWidget(lblPosX, r++, 1);
    grid->addWidget(new QLabel("Y (m):"), r, 0); grid->addWidget(lblPosY, r++, 1);
    grid->addWidget(new QLabel("Z (m):"), r, 0); grid->addWidget(lblPosZ, r++, 1);
    grid->addWidget(new QLabel("Roll (rad):"),  r, 0); grid->addWidget(lblRoll,  r++, 1);
    grid->addWidget(new QLabel("Pitch (rad):"), r, 0); grid->addWidget(lblPitch, r++, 1);
    grid->addWidget(new QLabel("Yaw (rad):"),   r, 0); grid->addWidget(lblYaw,   r++, 1);

    poseBox->setLayout(grid);

    statsWidget = new mtsQtWidgetIntervalStatistics();

    auto * layout = new QVBoxLayout;
    layout->addWidget(poseBox);
    layout->addWidget(statsWidget);
    setLayout(layout);

    setWindowTitle("PSMove Pose");
    resize(sizeHint());
}

void mtsPSMoveQtWidget::setPoseLabels(const prmPositionCartesianGet & cp)
{
    const vctFrm3 & F = cp.Position();
    lblPosX->setText(QString::number(F.Translation().X(), 'f', 3));
    lblPosY->setText(QString::number(F.Translation().Y(), 'f', 3));
    lblPosZ->setText(QString::number(F.Translation().Z(), 'f', 3));

    double r, p, y; rotToRPY(F.Rotation(), r, p, y);
    lblRoll ->setText(QString::number(r, 'f', 3));
    lblPitch->setText(QString::number(p, 'f', 3));
    lblYaw  ->setText(QString::number(y, 'f', 3));
}

void mtsPSMoveQtWidget::timerEvent(QTimerEvent *)
{
    // Pose
    auto res = Dev.MeasuredCP(Dev.CP);
    if (!res) {
        CMN_LOG_CLASS_RUN_ERROR << "Controller.measured_cp failed: " << res << std::endl;
    } else if (Dev.CP.Valid()) {
        setPoseLabels(Dev.CP);
    }

    // Interval stats
    mtsIntervalStatistics stats;
    Dev.GetPeriodStatistics(stats);
    statsWidget->SetValue(stats);
}
