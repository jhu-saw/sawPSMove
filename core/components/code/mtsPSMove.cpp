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
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstCommon/cmnLogger.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

extern "C" {
#include <psmoveapi/psmove.h>
#include <psmoveapi/psmove_tracker.h>
}


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPSMove,
                                      mtsTaskPeriodic,
                                      mtsTaskPeriodicConstructorArg);

mtsPSMove::mtsPSMove(const std::string & component_name, const double & period_in_seconds):
    mtsTaskPeriodic(component_name, period_in_seconds)
{
    initialize();
}


void mtsPSMove::initialize(void)
{
    m_operating_state.SetValid(true);
    m_operating_state.SetState(prmOperatingState::DISABLED);

    m_measured_cp.SetValid(false);
    m_measured_cp.SetMovingFrame("psmove");
    m_measured_cp.SetReferenceFrame("world");
    m_measured_cp.SetTimestamp(0.0);

    m_R_world_cam.Assign(vctMatRot3::Identity());
    m_t_world_cam.Assign(0.0, 0.0, 0.0);

    m_gripper_measured_js.SetValid(false);
    m_gripper_measured_js.Name().resize(1);
    m_gripper_measured_js.Name().at(0) = "gripper";
    m_gripper_measured_js.Position().SetSize(1);

    // predefined payloads
    m_event_payloads.pressed.SetType(prmEventButton::PRESSED);
    m_event_payloads.pressed.SetValid(true);
    m_event_payloads.released.SetType(prmEventButton::RELEASED);
    m_event_payloads.released.SetValid(true);

    // State table entries
    StateTable.AddData(m_operating_state, "operating_state");
    StateTable.AddData(m_measured_cp, "measured_cp"); // CRTK name
    StateTable.AddData(m_gripper_measured_js, "gripper_measured_js"); // CRTK name
    StateTable.AddData(m_accel,       "accel_raw");
    StateTable.AddData(m_gyro,        "gyro_raw");
    StateTable.AddData(m_trigger,     "trigger");
    StateTable.AddData(m_battery,     "battery");
    StateTable.AddData(m_buttons,     "buttons");

    const std::string _controller_name = "controller"; // ideally a loop for multiple controllers
    // Provided interface
    m_interface = AddInterfaceProvided(_controller_name);
    if (m_interface) {
        m_interface->AddMessageEvents();

        // CRTK-friendly command name
        m_interface->AddCommandReadState(StateTable, m_operating_state, "operating_state");
        m_interface->AddEventWrite(m_operating_state_event, "operating_state", prmOperatingState());
        m_interface->AddCommandWrite(&mtsPSMove::state_command,
                                     this, "state_command", std::string(""));

        m_interface->AddCommandReadState(StateTable, StateTable.PeriodStats,  "period_statistics");

        m_interface->AddCommandReadState(StateTable, m_measured_cp, "measured_cp");
        m_interface->AddCommandReadState(StateTable, m_gripper_measured_js, "gripper/measured_js");

        // Extra reads (handy in Qt and for debugging)
        m_interface->AddCommandReadState(StateTable, m_accel,   "get_accelerometer");
        m_interface->AddCommandReadState(StateTable, m_gyro,    "get_gyroscope");
        m_interface->AddCommandReadState(StateTable, m_trigger, "trigger");
        m_interface->AddCommandReadState(StateTable, m_battery, "battery");
        m_interface->AddCommandReadState(StateTable, m_buttons, "get_buttons");

        // Write commands
        m_interface->AddCommandWrite(&mtsPSMove::set_LED, this, "set_LED");
        m_interface->AddCommandWrite(&mtsPSMove::rumble, this, "rumble");
        m_interface->AddCommandVoid(&mtsPSMove::reset_orientation, this, "reset_orientation");

        // Camera / tracking commands
        m_interface->AddCommandWrite(&mtsPSMove::enable_camera, this, "enable_camera");
        m_interface->AddCommandWrite(&mtsPSMove::set_intrinsics, this, "set_intrinsics"); // fx,fy,cx,cy
        m_interface->AddCommandWrite(&mtsPSMove::set_camera_translation, this, "set_camera_translation");
        m_interface->AddCommandWrite(&mtsPSMove::set_camera_rotation, this, "set_camera_rotation");
    }

    // button interfaces
    mtsInterfaceProvided * button_interface;
    button_interface = AddInterfaceProvided(_controller_name + "/square");
    if (button_interface) {
        button_interface->AddEventWrite(m_square_event, "Button", prmEventButton());
    }
    button_interface = AddInterfaceProvided(_controller_name + "/triangle");
    if (button_interface) {
        button_interface->AddEventWrite(m_triangle_event, "Button", prmEventButton());
    }
    button_interface = AddInterfaceProvided(_controller_name + "/circle");
    if (button_interface) {
        button_interface->AddEventWrite(m_circle_event, "Button", prmEventButton());
    }
    button_interface = AddInterfaceProvided(_controller_name + "/cross");
    if (button_interface) {
        button_interface->AddEventWrite(m_cross_event, "Button", prmEventButton());
    }
    button_interface = AddInterfaceProvided(_controller_name + "/move");
    if (button_interface) {
        button_interface->AddEventWrite(m_move_event, "Button", prmEventButton());
    }

    // Initial camera status
    camera_set_status(CameraStatus::Disabled);
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
    camera_stop();
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
    // TODO: Ignore this code for now. It is disabled by hardcoding m_camera_requested to false in the header.
    // Enable camera / tracking if requested
    // if (args.find("camera:1") != std::string::npos) {
    //     m_camera_requested = true;
    // }
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

    // honor requested camera state
    if (m_camera_requested) {
        camera_start_if_needed();
    }
    
    // default dim cyan
    psmove_set_leds(m_move_handle, 0, 32, 32);
    psmove_update_leds(m_move_handle);

    // dummy state, should use state of PS controller
    m_operating_state.SetState(prmOperatingState::ENABLED);
    m_operating_state.SetIsHomed(true);
    m_operating_state_event(m_operating_state);

}


void mtsPSMove::Run(void)
{
    ProcessQueuedCommands();

    // Drive camera state machine
    const double now = osaGetTime();
    camera_step(now);

    if (!m_move_handle) {
        m_measured_cp.SetValid(false);
        m_gripper_measured_js.SetValid(false);
        return;
    }

    if (m_tracker_handle &&
        (m_camera_status == CameraStatus::Calibrating ||
         m_camera_status == CameraStatus::Ready)) {
        psmove_tracker_update_image(m_tracker_handle);
        psmove_tracker_update(m_tracker_handle, nullptr);
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


void mtsPSMove::state_command(const std::string & command)
{
    std::string humanReadableMessage;
    prmOperatingState::StateType newOperatingState;
    try {
        if (m_operating_state.ValidCommand(prmOperatingState::CommandTypeFromString(command),
                                           newOperatingState, humanReadableMessage)) {
            if (command == "enable") {
                if (m_move_handle) {
                    m_operating_state.SetState(prmOperatingState::ENABLED);
                }
            } else if (command == "disable") {
            } else if (command == "home") {
                m_operating_state.SetIsHomed(true);
            } else if (command == "unhome") {
                m_operating_state.SetIsHomed(false);
            } else if (command == "pause") {
            } else if (command == "resume") {
                return;
            }
        } else {
            m_interface->SendWarning(this->GetName() + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        m_interface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
    m_operating_state_event(m_operating_state);
}


void mtsPSMove::update_data(void)
{
    // Buttons
    unsigned int _new_buttons = psmove_get_buttons(m_move_handle);
    if (_new_buttons != m_buttons) {
        m_buttons = _new_buttons;
        bool _new_button;
        _new_button = m_buttons & Btn_SQUARE;
        if (_new_button != m_square_value) {
            m_square_event(_new_button ? m_event_payloads.pressed : m_event_payloads.released);
            m_square_value = _new_button;
        }
        _new_button = m_buttons & Btn_TRIANGLE;
        if (_new_button != m_triangle_value) {
            m_triangle_event(_new_button ? m_event_payloads.pressed : m_event_payloads.released);
            m_triangle_value = _new_button;
        }
        _new_button = m_buttons & Btn_CIRCLE;
        if (_new_button != m_circle_value) {
            m_circle_event(_new_button ? m_event_payloads.pressed : m_event_payloads.released);
            m_circle_value = _new_button;
        }
        _new_button = m_buttons & Btn_CROSS;
        if (_new_button != m_cross_value) {
            m_cross_event(_new_button ? m_event_payloads.pressed : m_event_payloads.released);
            m_cross_value = _new_button;
        }
        _new_button = m_buttons & Btn_MOVE;
        if (_new_button != m_move_value) {
            m_move_event(_new_button ? m_event_payloads.pressed : m_event_payloads.released);
            m_move_value = _new_button;
        }

        // buttons used to control mtsPSMove
        _new_button = m_buttons & Btn_SELECT;
        if (_new_button) {
            m_camera_requested = !m_camera_requested;
            enable_camera(m_camera_requested);
            m_interface->SendStatus(m_camera_requested ? "Camera enabled via \"select\" button" : "Camera disabled via \"select\" button");
        }
        _new_button = m_buttons & Btn_START;
        if (_new_button) {
            reset_orientation();
            m_interface->SendStatus("Orientation reset via Cross button");
        }
    }

    // Trigger & gripper
    uint8_t trig = psmove_get_trigger(m_move_handle);
    m_trigger = static_cast<double>(trig) / 255.0;
    m_gripper_measured_js.Position().at(0) = m_trigger;
    m_gripper_measured_js.SetValid(true);

    // Battery
    PSMove_Battery_Level b = psmove_get_battery(m_move_handle);
    int _new_battery;
    switch (b) {
        case Batt_MIN:       _new_battery = 5; break;
        case Batt_20Percent: _new_battery = 20; break;
        case Batt_40Percent: _new_battery = 40; break;
        case Batt_60Percent: _new_battery = 60; break;
        case Batt_80Percent: _new_battery = 80; break;
        case Batt_MAX:       _new_battery = 100; break;
        case Batt_CHARGING:  _new_battery = 99; break;
        default:             _new_battery = 0;  break;
    }
    if (_new_battery != m_battery) {
        if (_new_battery == 99) {
            m_interface->SendStatus("Battery is charging");
        } else {
            m_interface->SendStatus("Battery level is " + std::to_string(_new_battery) + "%");
        }
    }
    m_battery = _new_battery;

    // Raw IMU
    float ax, ay, az, gx, gy, gz;
    psmove_get_accelerometer_frame(m_move_handle, Frame_SecondHalf, &ax, &ay, &az);
    psmove_get_gyroscope_frame(m_move_handle, Frame_SecondHalf, &gx, &gy, &gz);
    m_accel.Assign(ax, ay, az);
    m_gyro.Assign(gx, gy, gz);

    // Orientation → rotation matrix
    // vctMatRot3 R;
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

    if (m_tracker_handle) {
        float u = 0.f, v = 0.f, r = 0.f;
        const int age_ms = psmove_tracker_get_position(m_tracker_handle, m_move_handle, &u, &v, &r);
        const bool ok_px = std::isfinite(u) && std::isfinite(v) && std::isfinite(r) && (r > 1e-6f) && (age_ms >= 0);

        if (ok_px) {
            // Seed (cx, cy) from the actual tracker image size once
            camera_init_image_center_from_tracker();
            const float dist_cm = psmove_tracker_distance_from_radius(m_tracker_handle, r);
            const double Z = double(dist_cm) * 0.01; // convert cm -> meters
            // provide a reasonable fx,fy once.
            int w = 0, h = 0; 
            psmove_tracker_get_size(m_tracker_handle, &w, &h);
            camera_init_fx_fy_if_needed(w, h);

            // Pinhole projection to get X,Y in meters (camera frame).
            const double X = (double(u) - m_cx) * Z / m_fx;
            const double Y = (double(v) - m_cy) * Z / m_fy;

            const vct3 p_cam(X, Y, Z);
            const vct3 p_w = m_R_world_cam * p_cam + m_t_world_cam;

            m_measured_cp.Position().Translation().Assign(p_w[0], p_w[1], p_w[2]);
            m_measured_cp.SetValid(true); 
        }
        // If not ok_px, we keep last translation to avoid output flicker on intermittent tracking.
    } else {
        // No tracker
        m_measured_cp.Position().Translation().Assign(0.0, 0.0, 0.0);
    }

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


void mtsPSMove::rumble(const double & strength)
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


void mtsPSMove::enable_camera(const bool &enable)
{
    m_camera_requested = enable;
    if (!enable) {
        camera_stop();
        return;
    }
    camera_start_if_needed();
}


void mtsPSMove::set_intrinsics(const vctDouble4 &fx_fy_cx_cy)
{
    m_fx = fx_fy_cx_cy[0];
    m_fy = fx_fy_cx_cy[1];
    m_cx = fx_fy_cx_cy[2];
    m_cy = fx_fy_cx_cy[3];
}


void mtsPSMove::set_camera_translation(const vctDouble3 &translation)
{
    m_t_world_cam = translation;
}


void mtsPSMove::set_camera_rotation(const vctMatRot3 &rotation)
{
    m_R_world_cam = rotation;
}


// ----------------- Camera state machine -----------------
void mtsPSMove::camera_set_status(const CameraStatus s)
{
    if (m_camera_status == s) {
        return;
    }
    m_camera_status = s;
    switch (s) {
        case CameraStatus::Disabled:    m_interface->SendStatus("Camera: disabled"); break;
        case CameraStatus::Starting:    m_interface->SendStatus("Camera: starting"); break;
        case CameraStatus::Calibrating: m_interface->SendStatus("Camera: calibrating"); break;
        case CameraStatus::Ready:       m_interface->SendStatus("Camera: ready"); break;
        case CameraStatus::Error:       m_interface->SendError("Camera: error"); break;
    }
}


void mtsPSMove::camera_start_if_needed()
{   
    if (!m_move_handle || m_tracker_handle) return; 
    
    const double now = osaGetTime();
    if ((now - m_cam_last_enable_try_sec) < m_cam_retry_period_sec) return;

    m_cam_last_enable_try_sec = now;

    // Check if any cameras/trackers are available before attempting to create one
    int tracker_count = psmove_tracker_count_connected(); // FIXME: This is useless. It lists all connected trackers, not if they are actually usable.
    if (tracker_count <= 0) {
        m_interface->SendWarning("Camera requested but no cameras detected. Continuing with orientation-only tracking.");
        m_camera_requested = false;  
        camera_set_status(CameraStatus::Disabled);
        return;
    }

    m_tracker_handle = psmove_tracker_new(); // BUG: If camera is requested on startup and there is no compatible camera, then the mtsComponent doesnt start/gets stuck.
    if (!m_tracker_handle) {
        m_interface->SendWarning("Camera requested but tracker creation failed. Continuing with orientation-only tracking.");
        m_camera_requested = false;  
        camera_set_status(CameraStatus::Disabled);
        return;
    }
    
    // >>> disable LED rate limiting during calibration <<<
    psmove_set_rate_limiting(m_move_handle, 0);

    camera_set_status(CameraStatus::Starting);
    // Kick calibration on next step
}


// Uses the tracker's actual image size to seed (cx, cy) once.
void mtsPSMove::camera_init_image_center_from_tracker()
{
    if (!m_tracker_handle) { return; }
    int w = 0, h = 0;
    psmove_tracker_get_size(m_tracker_handle, &w, &h); // ask tracker which image size it's using
    if (w > 0 && h > 0) {
        // avoid overwriting values set via set_intrinsics().
        if (!(std::isfinite(m_cx) && std::isfinite(m_cy) && m_cx != 0.0 && m_cy != 0.0)) {
            m_cx = 0.5 * double(w);  // assume principal point at image center
            m_cy = 0.5 * double(h);
        }
    }
}


// Provides a one-time sensible default for fx, fy if the user never sets intrinsics.
// used to scale X,Y from pixels to meters; Z comes from the tracker model.
void mtsPSMove::camera_init_fx_fy_if_needed(int width_px, [[maybe_unused]] int height_px)
{
    if (m_fx > 0.0 && m_fy > 0.0) return; // respect user-provided values

    // PS Eye wide FOV is ~75–80° horizontally. Use 75° as a conservative default.
    const double fov_h_deg = 75.0;
    const double fov_h_rad = fov_h_deg * (M_PI / 180.0);

    // fx ~ (W/2) / tan(FOV/2). Assume square pixels -> fy = fx.
    const double fx_guess = (0.5 * double(width_px)) / std::tan(0.5 * fov_h_rad);
    if (!(m_fx > 0.0)) m_fx = fx_guess;
    if (!(m_fy > 0.0)) m_fy = fx_guess;
}


void mtsPSMove::camera_step(double now_sec)
{
    // Honor desired state
    if (!m_camera_requested) {
        if (m_tracker_handle) camera_stop();
        return;
    }
    if (!m_tracker_handle) {
        camera_start_if_needed();
        return;
    }

    // Starting -> issue enable; if instant CALIBRATED, become Ready; else go Calibrating
    if (m_camera_status == CameraStatus::Starting) {
        auto st = psmove_tracker_enable(m_tracker_handle, m_move_handle);
        if (st == Tracker_CALIBRATED) {
            camera_set_status(CameraStatus::Ready);
            // >>> re-enable rate limiting once Ready <<<
            psmove_set_rate_limiting(m_move_handle, 1);
            return;
        }
        m_cam_calib_start_sec = now_sec;
        camera_set_status(CameraStatus::Calibrating);
        m_interface->SendStatus("Camera: calibrating");
        return;
    }

    // Calibrating -> wait until CALIBRATED or timeout; on timeout recreate later
    if (m_camera_status == CameraStatus::Calibrating) {
        auto st = psmove_tracker_enable(m_tracker_handle, m_move_handle);
        if (st == Tracker_CALIBRATED) {
            // write a message that we are calibrated
            m_interface->SendStatus("Camera: calibrated");
            camera_set_status(CameraStatus::Ready);
            return;
        }
        if ((now_sec - m_cam_calib_start_sec) > m_cam_calib_timeout_sec) {
            // Camera calibration failed - warn user and disable camera functionality
            m_interface->SendWarning("Camera calibration timeout. Continuing with orientation-only tracking.");
            m_camera_requested = false;  // Disable further camera attempts
            camera_stop();
            return;
        }
        return;
    }
}


void mtsPSMove::camera_stop(void)
{
    if (m_tracker_handle) {
        psmove_tracker_free(m_tracker_handle);
        m_tracker_handle = nullptr;
    }
    // Re-enable rate limiting when stopping camera
    if (m_move_handle) {
        psmove_set_rate_limiting(m_move_handle, 1);
    }
    camera_set_status(CameraStatus::Disabled);
    m_cam_last_enable_try_sec = 0.0;
    m_cam_calib_start_sec = 0.0;
}
