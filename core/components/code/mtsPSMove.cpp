// sawPSMove/components/code/mtsPSMove.cpp
#include <sawPSMove/mtsPSMove.h>

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstCommon/cmnLogger.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

extern "C" {
#include <psmoveapi/psmove.h>
}

CMN_IMPLEMENT_SERVICES(mtsPSMove)

mtsPSMove::mtsPSMove(const std::string & componentName)
: mtsTaskContinuous(componentName)
{
    Table = &StateTable;

    m_measured_cp.SetValid(false);
    m_measured_cp.SetMovingFrame("psmove");
    m_measured_cp.SetReferenceFrame("world");
    m_measured_cp.SetTimestamp(0.0);
    vctMatRot3 R; R.Identity();
    vct3 t(0.0);
    m_measured_cp.SetPosition(vctFrm3(R, t));

    // State table entries
    Table->AddData(m_measured_cp, "measured_cp"); // CRTK name
    Table->AddData(m_accel,       "accel_raw");
    Table->AddData(m_gyro,        "gyro_raw");
    Table->AddData(m_trigger,     "trigger");
    Table->AddData(m_battery,     "battery");
    Table->AddData(m_buttons,     "buttons");

    // Provided interface
    Interface = AddInterfaceProvided("controller");
    if (Interface) {
        // CRTK-friendly command name
        Interface->AddCommandReadState(*Table, m_measured_cp, "measured_cp");

        // Extra reads (handy in Qt and for debugging)
        Interface->AddCommandReadState(*Table, m_accel,       "GetAccelerometer");
        Interface->AddCommandReadState(*Table, m_gyro,        "GetGyroscope");
        Interface->AddCommandReadState(*Table, m_trigger,     "GetTrigger");
        Interface->AddCommandReadState(*Table, m_battery,     "GetBattery");
        Interface->AddCommandReadState(*Table, m_buttons,     "GetButtons");

        // Period stats like in OptoForce
        Interface->AddCommandReadState(*Table, Table->Period,       "GetTaskPeriod");
        Interface->AddCommandReadState(*Table, Table->PeriodStats,  "get_period_statistics");

        // Write commands
        Interface->AddCommandWrite(&mtsPSMove::SetLED, this, "SetLED");
        Interface->AddCommandWrite(&mtsPSMove::SetRumble, this, "SetRumble");
        Interface->AddCommandVoid(&mtsPSMove::ResetOrientation, this, "ResetOrientation");
    }
}

mtsPSMove::mtsPSMove(const std::string & componentName,
                     const std::string & connectionHint)
: mtsPSMove(componentName) // delegate
{
    Configure(connectionHint);
}

mtsPSMove::~mtsPSMove()
{
    if (Move) {
        psmove_set_rumble(Move, 0);
        psmove_set_leds(Move, 0, 0, 0);
        psmove_update_leds(Move);
        psmove_disconnect(Move);
        Move = nullptr;
    }
}

void mtsPSMove::Configure(const std::string &args)
{
    // Examples: "", "id:1", "1"
    if (args.empty()) { return; }
    auto pos = args.find("id:");
    if (pos != std::string::npos) {
        ControllerIndex = std::atoi(args.c_str() + pos + 3);
    } else {
        // numeric? then treat as index
        bool numeric = !args.empty() && std::strspn(args.c_str(), "0123456789") == args.size();
        if (numeric) {
            ControllerIndex = std::atoi(args.c_str());
        }
    }
}

void mtsPSMove::Startup(void)
{
    int count = psmove_count_connected();
    if (count <= 0) {
        CMN_LOG_CLASS_INIT_ERROR << "No PS Move controllers found" << std::endl;
        return;
    }
    if (ControllerIndex < 0 || ControllerIndex >= count) {
        CMN_LOG_CLASS_INIT_WARNING << "Controller index " << ControllerIndex
                                   << " out of range [0.." << (count-1) << "], using 0" << std::endl;
        ControllerIndex = 0;
    }

    Move = psmove_connect_by_id(ControllerIndex);
    if (!Move) {
        CMN_LOG_CLASS_INIT_ERROR << "Failed to connect to PS Move " << ControllerIndex << std::endl;
        return;
    }

    psmove_set_rate_limiting(Move, 1);

    if (psmove_has_calibration(Move)) {
        psmove_enable_orientation(Move, 1);
        OrientationAvailable = (psmove_has_orientation(Move) != 0);
    } else {
        OrientationAvailable = false;
        CMN_LOG_CLASS_INIT_WARNING << "No magnetometer calibration; orientation may be unavailable." << std::endl;
    }

    // default dim cyan
    psmove_set_leds(Move, 0, 32, 32);
    psmove_update_leds(Move);
}

void mtsPSMove::Run(void)
{
    ProcessQueuedCommands();

    if (!Move) {
        m_measured_cp.SetValid(false);
        osaSleep(0.005); // don’t spin
        return;
    }

    // poll all pending samples
    while (psmove_poll(Move)) {
        UpdateFromController();
    }

    // Timestamp from cisst time server
    m_measured_cp.SetTimestamp(mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime());
    Table->Advance();

    ProcessQueuedEvents();
}

void mtsPSMove::Cleanup(void)
{
    // handled in destructor
}

void mtsPSMove::UpdateFromController()
{
    // Buttons & trigger
    m_buttons = psmove_get_buttons(Move);
    uint8_t trig = psmove_get_trigger(Move);
    m_trigger = static_cast<double>(trig) / 255.0;

    // Battery
    PSMove_Battery_Level b = psmove_get_battery(Move);
    switch (b) {
        case Batt_MIN:       m_battery = 0.05; break;
        case Batt_20Percent: m_battery = 0.20; break;
        case Batt_40Percent: m_battery = 0.40; break;
        case Batt_60Percent: m_battery = 0.60; break;
        case Batt_80Percent: m_battery = 0.80; break;
        case Batt_MAX:
        case Batt_CHARGING:  m_battery = 1.00; break;
        default:             m_battery = 0.0;  break;
    }

    // Raw IMU
    float ax, ay, az, gx, gy, gz;
    psmove_get_accelerometer_frame(Move, Frame_SecondHalf, &ax, &ay, &az);
    psmove_get_gyroscope_frame(Move, Frame_SecondHalf, &gx, &gy, &gz);
    m_accel.Assign(ax, ay, az);
    m_gyro.Assign(gx, gy, gz);

    // Orientation → rotation matrix
    vctMatRot3 R; R.Identity();
    if (OrientationAvailable) {
        float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
        // NOTE: In this API variant, psmove_get_orientation returns void.
        //       We just call it and then sanity-check the result.
        psmove_get_orientation(Move, &qw, &qx, &qy, &qz);

        const bool ok =
            std::isfinite(qw) && std::isfinite(qx) &&
            std::isfinite(qy) && std::isfinite(qz) &&
            (qw*qw + qx*qx + qy*qy + qz*qz) > 1e-12;

        if (ok) {
            QuaternionToRotation(qw, qx, qy, qz, R);
            m_measured_cp.SetValid(true);
        } else {
            m_measured_cp.SetValid(false);
        }
    } else {
        m_measured_cp.SetValid(false);
    }

    // Translation unknown without camera tracker; keep zero
    vct3 t(0.0);
    m_measured_cp.SetPosition(vctFrm3(R, t));
}

void mtsPSMove::QuaternionToRotation(double w, double x, double y, double z, vctMatRot3 &R) const
{
    const double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n > 1e-12) { w/=n; x/=n; y/=n; z/=n; }

    const double xx = x*x, yy = y*y, zz = z*z;
    const double xy = x*y, xz = x*z, yz = y*z;
    const double wx = w*x, wy = w*y, wz = w*z;

    R.Element(0,0) = 1.0 - 2.0*(yy + zz);
    R.Element(0,1) = 2.0*(xy - wz);
    R.Element(0,2) = 2.0*(xz + wy);

    R.Element(1,0) = 2.0*(xy + wz);
    R.Element(1,1) = 1.0 - 2.0*(xx + zz);
    R.Element(1,2) = 2.0*(yz - wx);

    R.Element(2,0) = 2.0*(xz - wy);
    R.Element(2,1) = 2.0*(yz + wx);
    R.Element(2,2) = 1.0 - 2.0*(xx + yy);
}

void mtsPSMove::SetLED(const vctDouble3 &rgb)
{
    if (!Move) return;
    const uint8_t r = static_cast<uint8_t>(Clamp01(rgb[0]) * 255.0);
    const uint8_t g = static_cast<uint8_t>(Clamp01(rgb[1]) * 255.0);
    const uint8_t b = static_cast<uint8_t>(Clamp01(rgb[2]) * 255.0);
    psmove_set_leds(Move, r, g, b);
    psmove_update_leds(Move);
}

void mtsPSMove::SetRumble(const double & strength)
{
    if (!Move) return;
    const uint8_t s = static_cast<uint8_t>(Clamp01(strength) * 255.0);
    psmove_set_rumble(Move, s);
    psmove_update_leds(Move);
}

void mtsPSMove::ResetOrientation(void)
{
    if (!Move) return;
    psmove_reset_orientation(Move);
}
