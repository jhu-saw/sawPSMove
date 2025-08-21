// sawPSMove/components/include/sawPSMove/mtsPSMoveQtWidget.h
#pragma once

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <QObject>  // needed for Q_OBJECT/Q_SLOTS
#include <QWidget>
#include <QTimer>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QGridLayout>

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
    mtsFunctionRead GetPositionCartesian;
    mtsFunctionRead GetButtons;
    mtsFunctionRead GetTrigger;
    mtsFunctionRead GetPeriodStats;
    mtsFunctionRead GetBattery;
    mtsFunctionWrite SetLED;
    mtsFunctionWrite SetRumble;
    mtsFunctionVoid  ResetOrientation;

    // UI
    QTimer *Timer{nullptr};
    QLabel *PoseQ{nullptr};
    QLabel *PoseRPY{nullptr};
    QLabel *Buttons{nullptr};
    QLabel *Trigger{nullptr};
    QLabel *Battery{nullptr};
    QDoubleSpinBox *Rumble{nullptr};
    QDoubleSpinBox *LED_R{nullptr};
    QDoubleSpinBox *LED_G{nullptr};
    QDoubleSpinBox *LED_B{nullptr};
    QPushButton *LED_Set{nullptr};
    QPushButton *ResetOri{nullptr};

    // helpers
    static void RotationToRPY(const vctMatRot3 &R, double &r, double &p, double &y);
};
CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMoveQtWidget)
