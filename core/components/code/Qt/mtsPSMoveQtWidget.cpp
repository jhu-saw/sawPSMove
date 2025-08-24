/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Aravind S Kumar, Anton Deguet
  Created on: 2025-08-21
  
  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.
  
--- begin cisst license - do not edit ---
  
This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawPSMove/mtsPSMoveQtWidget.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidget.h>
#include <cisstParameterTypes/prmOperatingStateQtWidget.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTimer>

CMN_IMPLEMENT_SERVICES(mtsPSMoveQtWidget);

mtsPSMoveQtWidget::mtsPSMoveQtWidget(const std::string &name, QWidget *parent):
    QWidget(parent),
    mtsComponent(name)
{
    QMMessage = new mtsMessageQtWidget();
    // QPOState = new prmOperatingStateQtWidget();

    // Required interface to the device (OptoForce pattern)
    m_device_interface = AddInterfaceRequired("device");
    if (m_device_interface) {
        QMMessage->SetInterfaceRequired(m_device_interface);
        // QPOState->SetInterfaceRequired(m_device_interface);
        m_device_interface->AddFunction("measured_cp", device.measured_cp);
        m_device_interface->AddFunction("get_buttons", device.get_buttons);
        m_device_interface->AddFunction("get_trigger", device.get_trigger);
        m_device_interface->AddFunction("get_battery", device.get_battery);
        m_device_interface->AddFunction("set_LED", device.set_LED);
        m_device_interface->AddFunction("set_rumble", device.set_rumble);
        m_device_interface->AddFunction("reset_orientation", device.reset_orientation);
        m_device_interface->AddFunction("period_statistics", device.period_statistics);
    }

    auto *layout = new QGridLayout(this);
    Buttons   = new QLabel("buttons=0x00000000");
    Trigger   = new QLabel("trigger=0.00");
    Battery   = new QLabel("battery=0.00");

    Rumble = new QDoubleSpinBox(); Rumble->setRange(0.0, 1.0); Rumble->setSingleStep(0.05); Rumble->setValue(0.0);
    LED_R  = new QDoubleSpinBox(); LED_R->setRange(0.0, 1.0); LED_R->setSingleStep(0.1); LED_R->setValue(0.0);
    LED_G  = new QDoubleSpinBox(); LED_G->setRange(0.0, 1.0); LED_G->setSingleStep(0.1); LED_G->setValue(0.5);
    LED_B  = new QDoubleSpinBox(); LED_B->setRange(0.0, 1.0); LED_B->setSingleStep(0.1); LED_B->setValue(0.5);
    LED_Set = new QPushButton("Set LED");
    ResetOri = new QPushButton("Reset Orientation");

    int row = 0;

    QPCGWidget = new prmPositionCartesianGetQtWidget();
    layout->addWidget(QPCGWidget,   row++, 0, 1, 2);

    layout->addWidget(new QLabel("<b>Inputs</b>"), row++, 0, 1, 2);
    layout->addWidget(Buttons, row++, 0, 1, 2);
    layout->addWidget(Trigger, row++, 0, 1, 2);
    layout->addWidget(Battery, row++, 0, 1, 2);

    layout->addWidget(new QLabel("<b>Rumble</b>"), row, 0);
    layout->addWidget(Rumble, row++, 1);
    layout->addWidget(new QLabel("<b>LED (r,g,b)</b>"), row, 0);
    auto *ledRow = new QWidget; auto *ledLayout = new QHBoxLayout(ledRow);
    ledLayout->setContentsMargins(0,0,0,0);
    ledLayout->addWidget(LED_R); ledLayout->addWidget(LED_G); ledLayout->addWidget(LED_B);
    layout->addWidget(ledRow, row++, 1);
    layout->addWidget(LED_Set, row, 0);
    layout->addWidget(ResetOri, row++, 1);

    QMIntervalStatistics = new mtsIntervalStatisticsQtWidget();
    layout->addWidget(QMIntervalStatistics, row++, 0, 1, 2);

    QMMessage->setupUi();
    layout->addWidget(QMMessage, row++, 0, 1, 2);

    connect(LED_Set, &QPushButton::clicked, this, &mtsPSMoveQtWidget::OnSetLED);
    connect(Rumble, SIGNAL(valueChanged(double)), this, SLOT(OnRumbleChanged(double)));
    connect(ResetOri, &QPushButton::clicked, this, &mtsPSMoveQtWidget::OnResetOrientation);

    Timer = new QTimer(this);
    Timer->setInterval(33); // ~30 Hz UI
    connect(Timer, &QTimer::timeout, this, &mtsPSMoveQtWidget::OnTimer);
}


mtsPSMoveQtWidget::~mtsPSMoveQtWidget() {}


void mtsPSMoveQtWidget::Startup(void)
{
    Timer->start();
}


void mtsPSMoveQtWidget::Cleanup(void)
{
    Timer->stop();
}


void mtsPSMoveQtWidget::OnTimer()
{
    prmPositionCartesianGet cp;
    if (device.measured_cp(cp).IsOK()) {
        QPCGWidget->SetValue(cp);
    }

    unsigned int btn = 0;
    if (device.get_buttons(btn).IsOK()) {
        Buttons->setText(QString("buttons=0x%1").arg(QString::number(btn, 16).rightJustified(8, '0')));
    }
    double trig = 0.0;
    if (device.get_trigger(trig).IsOK()) {
        Trigger->setText(QString("trigger=%1").arg(trig, 0, 'f', 2));
    }
    int bat = 0.0;
    if (device.get_battery(bat).IsOK()) {
        Battery->setText(QString("battery=%1").arg(bat));
    }

    mtsIntervalStatistics _stats;
    device.period_statistics(_stats);
    QMIntervalStatistics->SetValue(_stats);

}


void mtsPSMoveQtWidget::OnSetLED()
{
    vctDouble3 rgb(LED_R->value(), LED_G->value(), LED_B->value());
    device.set_LED(rgb);
}


void mtsPSMoveQtWidget::OnRumbleChanged(double v)
{
    device.set_rumble(v);
}


void mtsPSMoveQtWidget::OnResetOrientation()
{
    device.reset_orientation();
}
