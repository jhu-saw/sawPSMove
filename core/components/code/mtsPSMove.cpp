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
    m_measured_cp.SetValid(false);
    m_measured_cp.SetMovingFrame("psmove");
    m_measured_cp.SetReferenceFrame("world");
    m_measured_cp.SetTimestamp(0.0);

    // State table entries
    StateTable.AddData(m_measured_cp, "measured_cp"); // CRTK name
    StateTable.AddData(m_accel,       "accel_raw");
    StateTable.AddData(m_gyro,        "gyro_raw");
    StateTable.AddData(m_trigger,     "trigger");
    StateTable.AddData(m_battery,     "battery");
    StateTable.AddData(m_buttons,     "buttons");

    // Provided interface
    m_interface = AddInterfaceProvided("controller");
    if (m_interface) {

        m_interface->AddMessageEvents();

        // CRTK-friendly command name
        m_interface->AddCommandReadState(StateTable, m_measured_cp, "measured_cp");

        // Extra reads (handy in Qt and for debugging)
        m_interface->AddCommandReadState(StateTable, m_accel,       "get_accelerometer");
        m_interface->AddCommandReadState(StateTable, m_gyro,        "get_gyroscope");
        m_interface->AddCommandReadState(StateTable, m_trigger,     "get_trigger");
        m_interface->AddCommandReadState(StateTable, m_battery,     "get_battery");
        m_interface->AddCommandReadState(StateTable, m_buttons,     "get_buttons");

        // Period stats
        m_interface->AddCommandReadState(StateTable, StateTable.PeriodStats,  "period_statistics");

        // Write commands
        m_interface->AddCommandWrite(&mtsPSMove::set_LED, this, "set_LED");
        m_interface->AddCommandWrite(&mtsPSMove::set_rumble, this, "set_rumble");
        m_interface->AddCommandVoid(&mtsPSMove::reset_orientation, this, "reset_orientation");
    }
}


mtsPSMove::~mtsPSMove()
{
    if (m_move_handle) {
        psmove_set_rumble(m_move_handle, 0);
        psmove_set_leds(m_move_handle, 0, 0, 0);
        psmove_update_leds(m_move_handle);
        psmove_disconnect(m_move_handle);
        m_move_handle = nullptr;
    }
}


void mtsPSMove::Configure(const std::string &args)
{
    // Examples: "", "id:1", "1"
    if (args.empty()) { return; }
    auto pos = args.find("id:");
    if (pos != std::string::npos) {
        m_controller_index = std::atoi(args.c_str() + pos + 3);
    } else {
        // numeric? then treat as index
        bool numeric = !args.empty() && std::strspn(args.c_str(), "0123456789") == args.size();
        if (numeric) {
            m_controller_index = std::atoi(args.c_str());
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
    if (m_controller_index < 0 || m_controller_index >= count) {
        CMN_LOG_CLASS_INIT_WARNING << "Controller index " << m_controller_index
                                   << " out of range [0.." << (count-1) << "], using 0" << std::endl;
        m_controller_index = 0;
    }

    m_move_handle = psmove_connect_by_id(m_controller_index);
    if (!m_move_handle) {
        CMN_LOG_CLASS_INIT_ERROR << "Failed to connect to PS Move " << m_controller_index << std::endl;
        return;
    }

    psmove_set_rate_limiting(m_move_handle, 1);

    if (psmove_has_calibration(m_move_handle)) {
        psmove_enable_orientation(m_move_handle, 1);
        m_orientation_available = (psmove_has_orientation(m_move_handle) != 0);
    } else {
        m_orientation_available = false;
        CMN_LOG_CLASS_INIT_WARNING << "No magnetometer calibration; orientation may be unavailable." << std::endl;
    }

    m_interface->SendStatus("Testing");

    // default dim cyan
    psmove_set_leds(m_move_handle, 0, 32, 32);
    psmove_update_leds(m_move_handle);
}


void mtsPSMove::Run(void)
{
    ProcessQueuedCommands();

    if (!m_move_handle) {
        m_measured_cp.SetValid(false);
        osaSleep(0.005); // don’t spin
        return;
    }

    // poll all pending samples
    while (psmove_poll(m_move_handle)) {
        update_data();
    }
}


void mtsPSMove::Cleanup(void)
{
    // handled in destructor
}


void mtsPSMove::update_data(void)
{
    // Buttons & trigger
    m_buttons = psmove_get_buttons(m_move_handle);
    uint8_t trig = psmove_get_trigger(m_move_handle);
    m_trigger = static_cast<double>(trig) / 255.0;

    // Battery
    PSMove_Battery_Level b = psmove_get_battery(m_move_handle);
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
    psmove_get_accelerometer_frame(m_move_handle, Frame_SecondHalf, &ax, &ay, &az);
    psmove_get_gyroscope_frame(m_move_handle, Frame_SecondHalf, &gx, &gy, &gz);
    m_accel.Assign(ax, ay, az);
    m_gyro.Assign(gx, gy, gz);

    // Orientation → rotation matrix
    vctMatRot3 R;
    if (m_orientation_available) {
        float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
        // NOTE: In this API variant, psmove_get_orientation returns void.
        //       We just call it and then sanity-check the result.
        psmove_get_orientation(m_move_handle, &qw, &qx, &qy, &qz);

        const bool ok =
            std::isfinite(qw) && std::isfinite(qx) &&
            std::isfinite(qy) && std::isfinite(qz) &&
            (qw*qw + qx*qx + qy*qy + qz*qz) > 1e-12;

        if (ok) {
            m_measured_cp.Position().Rotation().FromNormalized(vctQuaternionRotation3(qw, qx, qy, qz));
            m_measured_cp.SetValid(true);
        } else {
            m_measured_cp.SetValid(false);
        }
    } else {
        m_measured_cp.SetValid(false);
    }

    // Translation unknown without camera tracker; keep zero
    m_measured_cp.Position().Translation() = vct3(0.0);
}


void mtsPSMove::set_LED(const vctDouble3 &rgb)
{
    if (!m_move_handle) return;
    const uint8_t r = static_cast<uint8_t>(std::clamp(rgb[0], 0.0, 1.0) * 255.0);
    const uint8_t g = static_cast<uint8_t>(std::clamp(rgb[1], 0.0, 1.0) * 255.0);
    const uint8_t b = static_cast<uint8_t>(std::clamp(rgb[2], 0.0, 1.0) * 255.0);
    psmove_set_leds(m_move_handle, r, g, b);
    psmove_update_leds(m_move_handle);
}


void mtsPSMove::set_rumble(const double & strength)
{
    if (!m_move_handle) return;
    const uint8_t s = static_cast<uint8_t>(std::clamp(strength, 0.0, 1.0) * 255.0);
    psmove_set_rumble(m_move_handle, s);
    psmove_update_leds(m_move_handle);
}


void mtsPSMove::reset_orientation(void)
{
    if (!m_move_handle) return;
    psmove_reset_orientation(m_move_handle);
}
