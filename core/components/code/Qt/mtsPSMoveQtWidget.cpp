// sawPSMove/components/code/Qt/mtsPSMoveQtWidget.cpp
#include <sawPSMove/mtsPSMoveQtWidget.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstVector/vctMatrixRotation3.h>
#include <cisstVector/vctQuaternionRotation3.h>

#include <QHBoxLayout>
#include <cmath>

CMN_IMPLEMENT_SERVICES(mtsPSMoveQtWidget)

mtsPSMoveQtWidget::mtsPSMoveQtWidget(const std::string &name, QWidget *parent)
: QWidget(parent)
, mtsComponent(name)
{
    // Required interface to the device (OptoForce pattern)
    mtsInterfaceRequired *req = AddInterfaceRequired("device");
    if (req) {
        req->AddFunction("measured_cp",       GetPositionCartesian);
        req->AddFunction("GetButtons",        GetButtons);
        req->AddFunction("GetTrigger",        GetTrigger);
        req->AddFunction("GetBattery",        GetBattery);
        req->AddFunction("SetLED",            SetLED);
        req->AddFunction("SetRumble",         SetRumble);
        req->AddFunction("ResetOrientation",  ResetOrientation);
        req->AddFunction("get_period_statistics", GetPeriodStats);
    }

    auto *layout = new QGridLayout(this);
    PoseQ     = new QLabel("q=[0,0,0,1]");
    PoseRPY   = new QLabel("rpy=[0,0,0] deg");
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
    layout->addWidget(new QLabel("<b>Orientation</b>"), row++, 0, 1, 2);
    layout->addWidget(PoseQ,   row++, 0, 1, 2);
    layout->addWidget(PoseRPY, row++, 0, 1, 2);

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
    if (GetPositionCartesian(cp).IsOK()) {
        const vctMatRot3 &R = cp.Position().Rotation();
        vctQuatRot3 q(R, VCT_NORMALIZE);
        PoseQ->setText(QString("q=[%1, %2, %3, %4]")
            .arg(q.X(), 0, 'f', 3).arg(q.Y(), 0, 'f', 3)
            .arg(q.Z(), 0, 'f', 3).arg(q.W(), 0, 'f', 3));

        double roll, pitch, yaw;
        RotationToRPY(R, roll, pitch, yaw);
        PoseRPY->setText(QString("rpy=[%1, %2, %3] deg")
            .arg(roll * 180.0/M_PI, 0, 'f', 1)
            .arg(pitch* 180.0/M_PI, 0, 'f', 1)
            .arg(yaw  * 180.0/M_PI, 0, 'f', 1));
    }

    unsigned int btn = 0;
    if (GetButtons(btn).IsOK()) {
        Buttons->setText(QString("buttons=0x%1").arg(QString::number(btn, 16).rightJustified(8, '0')));
    }
    double trig = 0.0;
    if (GetTrigger(trig).IsOK()) {
        Trigger->setText(QString("trigger=%1").arg(trig, 0, 'f', 2));
    }
    double bat = 0.0;
    if (GetBattery(bat).IsOK()) {
        Battery->setText(QString("battery=%1").arg(bat, 0, 'f', 2));
    }
}

void mtsPSMoveQtWidget::OnSetLED()
{
    vctDouble3 rgb(LED_R->value(), LED_G->value(), LED_B->value());
    SetLED(rgb);
}
void mtsPSMoveQtWidget::OnRumbleChanged(double v)
{
    SetRumble(v);
}
void mtsPSMoveQtWidget::OnResetOrientation()
{
    ResetOrientation();
}

void mtsPSMoveQtWidget::RotationToRPY(const vctMatRot3 &R, double &r, double &p, double &y)
{
    p = std::asin(-R[2][0]);
    if (std::cos(p) > 1e-6) {
        r = std::atan2(R[2][1], R[2][2]);
        y = std::atan2(R[1][0], R[0][0]);
    } else {
        r = std::atan2(-R[1][2], R[1][1]);
        y = 0.0;
    }
}
