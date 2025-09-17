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
#include <psmoveapi/psmove_fusion.h>
}


// Utilities for OneEuroFilter
typedef double TimeStamp; // in seconds
static const TimeStamp UndefinedTime = -1.0;

class LowPassFilter
{
    double y, a, s;
    bool initialized;

    void setAlpha(double alpha) {
        if (alpha <= 0.0 || alpha > 1.0) {
            a = 0.5; // fallback
        } else {
            a = alpha;
        }
    }

public:
    LowPassFilter(double alpha, double initval=0.0)
        : y(initval), s(initval), initialized(false) {
        setAlpha(alpha);
    }

    double filter(double value) {
        double result;
        if (initialized) {
            result = a * value + (1.0 - a) * s;
        } else {
            result = value;
            initialized = true;
        }
        y = value;
        s = result;
        return result;
    }

    double filterWithAlpha(double value, double alpha) {
        setAlpha(alpha);
        return filter(value);
    }

    bool hasLastRawValue() const { return initialized; }
    double lastRawValue()   const { return y; }
    double lastFilteredValue() const { return s; }
};

class OneEuroFilter
{
    double freq;
    double mincutoff;
    double beta_;
    double dcutoff;
    LowPassFilter *x;
    LowPassFilter *dx;
    TimeStamp lasttime;

    double alpha(double cutoff) const {
        double te = 1.0 / freq;
        double tau = 1.0 / (2 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / te);
    }

public:
    OneEuroFilter(double freq,
                  double mincutoff=1.0,
                  double beta_=0.0,
                  double dcutoff=1.0)
        : x(nullptr), dx(nullptr), lasttime(UndefinedTime) {
        setFrequency(freq);
        setMinCutoff(mincutoff);
        setBeta(beta_);
        setDerivateCutoff(dcutoff);
        x  = new LowPassFilter(alpha(mincutoff));
        dx = new LowPassFilter(alpha(dcutoff));
    }

    ~OneEuroFilter() {
        delete x;
        delete dx;
    }

    double filter(double value, TimeStamp timestamp=UndefinedTime) {
        if (lasttime != UndefinedTime && timestamp != UndefinedTime && timestamp > lasttime) {
            freq = 1.0 / (timestamp - lasttime);
        }
        lasttime = timestamp;

        double dvalue = x->hasLastRawValue() ? (value - x->lastFilteredValue()) * freq : 0.0;
        double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));
        double cutoff = mincutoff + beta_ * fabs(edvalue);
        return x->filterWithAlpha(value, alpha(cutoff));
    }

    void setFrequency(double f)      { freq = (f > 0 ? f : 120.0); }
    void setMinCutoff(double mc)     { mincutoff = (mc > 0 ? mc : 1.0); }
    void setBeta(double b)           { beta_ = b; }
    void setDerivateCutoff(double dc){ dcutoff = (dc > 0 ? dc : 1.0); }
};



CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPSMove,
                                      mtsTaskPeriodic,
                                      mtsTaskPeriodicConstructorArg);

class mtsPSMoveController {

public:

    mtsPSMoveController(mtsPSMove * system,
                        PSMove * move_handle,
                        int index,
                        const std::string & name,
                        const std::string & serial,
                        mtsInterfaceProvided * interface_provided,
                        mtsStateTable * state_table):
        m_system(system),
        m_move_handle(move_handle),
        m_index(index),
        m_name(name),
        m_serial(serial),
        m_interface(interface_provided),
        m_state_table(state_table)
    {
        initialize();
    }


    // for logs
    const cmnClassServicesBase * Services(void) const {
        return m_system->Services();
    }
    inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
        return cmnLogger::GetMultiplexer();
    }


    void initialize(void) {
        // PS settings
        psmove_set_rate_limiting(m_move_handle, 1);
        // pick different colors
        psmove_set_leds(m_move_handle,
                        m_index % 3 * 50,
                        (m_index + 1) % 3 * 50,
                        (m_index + 2) % 3 * 50);
        psmove_update_leds(m_move_handle);

        if (psmove_has_calibration(m_move_handle)) {
            psmove_enable_orientation(m_move_handle, 1);
            m_orientation_available = (psmove_has_orientation(m_move_handle) != 0);
        } else {
            m_orientation_available = false;
            CMN_LOG_CLASS_INIT_WARNING << "Controller " << m_name
                                       << ": no magnetometer calibration, orientation may be unavailable" << std::endl;
        }

        m_measured_cp_local.SetValid(false);
        // m_measured_cp_local.SetMovingFrame(m_name);
        // m_measured_cp_local.SetReferenceFrame(m_system->m_reference_frame);
        m_measured_cp_local.SetTimestamp(0.0);

        m_measured_cp.SetValid(false);
        m_measured_cp.SetMovingFrame(m_name);
        m_measured_cp.SetReferenceFrame(m_system->m_reference_frame);
        m_measured_cp.SetTimestamp(0.0);

        m_gripper_measured_js.SetValid(false);
        m_gripper_measured_js.Name().resize(1);
        m_gripper_measured_js.Name().at(0) = "gripper";
        m_gripper_measured_js.Position().SetSize(1);

        m_state_table->AddData(m_measured_cp, "measured_cp"); // CRTK name
        m_state_table->AddData(m_gripper_measured_js, "gripper_measured_js"); // CRTK name
        m_state_table->AddData(m_accel, "accel_raw");
        m_state_table->AddData(m_gyro, "gyro_raw");
        m_state_table->AddData(m_trigger, "trigger");
        m_state_table->AddData(m_battery, "battery");
        m_state_table->AddData(m_buttons, "buttons");

        // turn on messages
        m_interface->AddMessageEvents();

        // system commands (replicated)
        m_interface->AddCommandReadState(m_system->StateTable,
                                         m_system->m_operating_state, "operating_state");
        m_interface->AddEventWrite(m_operating_state_event, "operating_state", prmOperatingState());
        m_interface->AddCommandWrite(&mtsPSMove::state_command,
                                     m_system, "state_command", std::string(""));
        m_interface->AddCommandReadState(m_system->StateTable,
                                         m_system->StateTable.PeriodStats,  "period_statistics");

        // controller commands
        m_interface->AddCommandReadState(*m_state_table, m_measured_cp, "measured_cp");
        m_interface->AddCommandReadState(*m_state_table, m_gripper_measured_js, "gripper/measured_js");

        // Extra reads (handy in Qt and for debugging)
        m_interface->AddCommandReadState(*m_state_table, m_accel, "accelerometer");
        m_interface->AddCommandReadState(*m_state_table, m_gyro, "gyroscope");
        m_interface->AddCommandReadState(*m_state_table, m_trigger, "trigger");
        m_interface->AddCommandReadState(*m_state_table, m_battery, "battery");
        m_interface->AddCommandReadState(*m_state_table, m_buttons, "get_buttons");

        // Write commands
        m_interface->AddCommandWrite(&mtsPSMoveController::set_LED, this, "set_LED");
        m_interface->AddCommandWrite(&mtsPSMoveController::rumble, this, "rumble");
        m_interface->AddCommandVoid(&mtsPSMoveController::reset_orientation, this, "reset_orientation");
    }


    void startup(void)
    {
        m_interface->SendStatus(m_name + ": serial number " + m_serial);
    }


    void destruct(void)
    {   
        psmove_set_rumble(m_move_handle, 0);
        psmove_set_leds(m_move_handle, 0, 0, 0);
        psmove_update_leds(m_move_handle);
        psmove_disconnect(m_move_handle);
        m_move_handle = nullptr;
    }


    void update_data(void)
    {
        while (psmove_poll(m_move_handle)) {

            // Buttons
            unsigned int _new_buttons = psmove_get_buttons(m_move_handle);
            if (_new_buttons != m_buttons) {
                m_buttons = _new_buttons;
                bool _new_button;
                _new_button = m_buttons & Btn_SQUARE;
                if (_new_button != m_square_value) {
                    m_square_event(_new_button ? prmEventButton::BUTTON_PRESSED : prmEventButton::BUTTON_RELEASED);
                    m_square_value = _new_button;
                }
                _new_button = m_buttons & Btn_TRIANGLE;
                if (_new_button != m_triangle_value) {
                    m_triangle_event(_new_button ? prmEventButton::BUTTON_PRESSED : prmEventButton::BUTTON_RELEASED);
                    m_triangle_value = _new_button;
                }
                _new_button = m_buttons & Btn_CIRCLE;
                if (_new_button != m_circle_value) {
                    m_circle_event(_new_button ? prmEventButton::BUTTON_PRESSED : prmEventButton::BUTTON_RELEASED);
                    m_circle_value = _new_button;
                }
                _new_button = m_buttons & Btn_CROSS;
                if (_new_button != m_cross_value) {
                    m_cross_event(_new_button ? prmEventButton::BUTTON_PRESSED : prmEventButton::BUTTON_RELEASED);
                    m_cross_value = _new_button;
                }
                _new_button = m_buttons & Btn_MOVE;
                if (_new_button != m_move_value) {
                    m_move_event(_new_button ? prmEventButton::BUTTON_PRESSED : prmEventButton::BUTTON_RELEASED);
                    m_move_value = _new_button;
                }

                // buttons used to control mtsPSMove
                _new_button = m_buttons & Btn_SELECT;
                if (_new_button) {
                    auto & s_camera_requested = m_system->m_camera_requested;
                    s_camera_requested = !s_camera_requested;
                    m_system->enable_camera(s_camera_requested);
                    m_interface->SendStatus(m_name + ": camera "
                                            + (s_camera_requested ? "enabled" : "disabled")
                                            + " triggered by \"select\" button");
                }
                _new_button = m_buttons & Btn_START;
                if (_new_button) {
                    reset_orientation();
                    m_interface->SendStatus(m_name + ": orientation reset triggered by \"start\" button");
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
                    m_interface->SendStatus(m_name + ": battery is charging");
                } else {
                    m_interface->SendStatus(m_name + ": battery level is " + std::to_string(_new_battery) + "%");
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
                    m_measured_cp_local.Position().Rotation().FromNormalized(vctQuaternionRotation3(qx, qy, qz, qw));
                    m_measured_cp_local.SetValid(true);
                } else {
                    m_measured_cp_local.SetValid(false);
                }
            } else {
                m_measured_cp_local.SetValid(false);
            }
        }
    }


    void set_LED(const vctDouble3 & rgb)
    {
        if (!m_move_handle) {
            return;
        }
        const uint8_t r = static_cast<uint8_t>(std::clamp(rgb[0], 0.0, 1.0) * 255.0);
        const uint8_t g = static_cast<uint8_t>(std::clamp(rgb[1], 0.0, 1.0) * 255.0);
        const uint8_t b = static_cast<uint8_t>(std::clamp(rgb[2], 0.0, 1.0) * 255.0);
        psmove_set_leds(m_move_handle, r, g, b);
        psmove_update_leds(m_move_handle);
    }


    void rumble(const double & strength)
    {
        if (!m_move_handle) {
            return;
        }
        const uint8_t s = static_cast<uint8_t>(std::clamp(strength, 0.0, 1.0) * 255.0);
        psmove_set_rumble(m_move_handle, s);
        psmove_update_leds(m_move_handle);
    }


    void reset_orientation(void)
    {
        if (!m_move_handle) {
            return;
        }
        psmove_reset_orientation(m_move_handle); // This would reset the local orientation 
    }

    // Data from system
    mtsPSMove * m_system = nullptr;
    PSMove * m_move_handle = nullptr;
    int m_index = -1;
    std::string m_name;
    std::string m_serial;
    mtsInterfaceProvided * m_interface = nullptr;
    mtsStateTable * m_state_table = nullptr;

    prmPositionCartesianGet m_measured_cp;    // Pose 
    prmPositionCartesianGet m_measured_cp_local; // Local pose before base frame transform
    prmStateJoint m_gripper_measured_js;
    vctDouble3 m_accel;                       // accel raw
    vctDouble3 m_gyro;                        // gyro raw
    unsigned int m_buttons{0};                // button mask
    double m_trigger{0.0};                    // 0..1
    int m_battery{0};                         // 0..100, 99 for charging (approx)
    bool m_orientation_available = false;

    mtsFunctionWrite m_operating_state_event;

    // Buttons
    bool m_square_value = false;
    mtsFunctionWrite m_square_event;
    bool m_triangle_value = false;
    mtsFunctionWrite m_triangle_event;
    bool m_circle_value = false;
    mtsFunctionWrite m_circle_event;
    bool m_cross_value = false;
    mtsFunctionWrite m_cross_event;
    bool m_move_value = false;
    mtsFunctionWrite m_move_event;

    // One Euro filters per axis (meters)
    OneEuroFilter m_filter_x{120.0, 1.2, 0.02, 1.0};
    OneEuroFilter m_filter_y{120.0, 1.2, 0.02, 1.0};
    OneEuroFilter m_filter_z{120.0, 1.2, 0.02, 1.0};
    bool          m_filters_enabled = true;

    // tiny deadband to kill sub-mm shimmer at rest
    double        m_deadband_m = 0.0009; // 0.9 mm
};


mtsPSMove::mtsPSMove(const std::string & component_name, const double & period_in_seconds):
    mtsTaskPeriodic(component_name, period_in_seconds)
{
    initialize();
}


void mtsPSMove::initialize(void)
{
    m_operating_state.SetValid(true);
    m_operating_state.SetState(prmOperatingState::DISABLED);

    // State table entries
    StateTable.AddData(m_operating_state, "operating_state");
    StateTable.AddData(m_measured_cp_array, "measured_cp_array");

    // Provided interface
    m_interface = AddInterfaceProvided("system");
    if (m_interface) {
        m_interface->AddMessageEvents();
        // CRTK-friendly command name
        m_interface->AddCommandReadState(StateTable, m_operating_state, "operating_state");
        m_interface->AddEventWrite(m_operating_state_event, "operating_state", prmOperatingState());
        m_interface->AddCommandWrite(&mtsPSMove::state_command,
                                     this, "state_command", std::string(""));
        m_interface->AddCommandReadState(StateTable, StateTable.PeriodStats,  "period_statistics");

        // Camera / tracking commands
        m_interface->AddCommandWrite(&mtsPSMove::enable_camera, this, "enable_camera");

        // All positions
        m_interface->AddCommandReadState(StateTable, m_measured_cp_array, "measured_cp_array");
    }

    // Initial camera status
    camera_set_status(CameraStatus::Disabled);
}


mtsPSMove::~mtsPSMove()
{
    if (m_fusion_handle) {
        psmove_fusion_free(m_fusion_handle);
    }
    for (auto controller : m_controllers) {
        controller->destruct();
    }
    camera_stop();
}


void mtsPSMove::Configure(const std::string & filename)
{
    if (!filename.empty()) {

        std::ifstream json_stream;
        json_stream.open(filename.c_str());

        Json::Value json_config, json_value;
        Json::Reader json_reader;
        // make sure the file valid json
        if (!json_reader.parse(json_stream, json_config)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                     << "File: " << filename << std::endl << "Error(s):" << std::endl
                                     << json_reader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }

        // configuration
        json_value = json_config["camera_requested"];
        if (!json_value.empty()) {
            m_camera_requested = json_value.asBool();
        }
        json_value = json_config["desired_controller_names"];
        if (json_value.isArray()) {
            for (Json::Value::ArrayIndex i = 0; i < json_value.size(); ++i) {
                m_desired_controller_names.push_back(json_value[i].asString());
            }
        }
        json_value = json_config["base_frame"];
        if (!json_config.empty()) {
            m_reference_frame = json_value["reference_frame"].asString();
            vctFrm4x4 _transform;
            cmnDataDeSerializeTextJSON<vctFrm4x4>(_transform, json_value["transform"]);
            m_base_frame.FromNormalized(_transform);
        }
    }

    size_t count = psmove_count_connected();
    if (count <= 0) {
        CMN_LOG_CLASS_INIT_ERROR << "No PS Move controllers found" << std::endl;
        return;
    }
    for (size_t index = 0; index < count; ++index) {
        PSMove * _move_handle = psmove_connect_by_id(index);
        if (!_move_handle) {
            CMN_LOG_CLASS_INIT_ERROR << "Failed to connect controller #" << index << std::endl;
        } else {
            char *serial = psmove_get_serial(_move_handle);
            std::string _serial(serial);
            psmove_free_mem(serial);
            std::string _name;
            if (index < m_desired_controller_names.size()) {
                _name = m_desired_controller_names.at(index);
            } else {
                _name = "controller" + std::to_string(index + 1);
            }
            mtsInterfaceProvided * _interface = AddInterfaceProvided(_name);
            mtsStateTable * _state_table = new mtsStateTable(500, _name);
            this->AddStateTable(_state_table);
            auto _new_controller = new mtsPSMoveController(this, _move_handle, index, _name, _serial,
                                                           _interface, _state_table);
            m_controllers.push_back(_new_controller);
        }
    }

    // Add button interfaces
    for (auto controller : m_controllers) {
        const std::string _controller_name = controller->m_name;
        mtsInterfaceProvided * button_interface;
        button_interface = AddInterfaceProvided(_controller_name + "/square");
        if (button_interface) {
            button_interface->AddEventWrite(controller->m_square_event, "Button", prmEventButton());
        }
        button_interface = AddInterfaceProvided(_controller_name + "/triangle");
        if (button_interface) {
            button_interface->AddEventWrite(controller->m_triangle_event, "Button", prmEventButton());
        }
        button_interface = AddInterfaceProvided(_controller_name + "/circle");
        if (button_interface) {
            button_interface->AddEventWrite(controller->m_circle_event, "Button", prmEventButton());
        }
        button_interface = AddInterfaceProvided(_controller_name + "/cross");
        if (button_interface) {
            button_interface->AddEventWrite(controller->m_cross_event, "Button", prmEventButton());
        }
        button_interface = AddInterfaceProvided(_controller_name + "/move");
        if (button_interface) {
            button_interface->AddEventWrite(controller->m_move_event, "Button", prmEventButton());
        }
    }

    // initialize array of poses
    m_measured_cp_array.Positions().resize(m_controllers.size());
    m_measured_cp_array.SetValid(true);

    // TODO: Ignore this code for now. It is disabled by hardcoding m_camera_requested to false in the header.
    // Enable camera / tracking if requested
    // if (args.find("camera:1") != std::string::npos) {
    //     m_camera_requested = true;
    // }
}


void mtsPSMove::Startup(void)
{
    for (auto controller : m_controllers) {
        controller->startup();
    }

    // honor requested camera state
    if (m_camera_requested) {
        camera_start_if_needed();
    }

    // dummy state
    m_operating_state.SetState(prmOperatingState::ENABLED);
    m_operating_state.SetIsHomed(true);
    dispatch_operating_state();
}


void mtsPSMove::Run(void)
{
    ProcessQueuedCommands();
    update_data();
}


void mtsPSMove::Cleanup(void)
{
    // handled in destructor
}


void mtsPSMove::get_controller_names(std::list<std::string> & controllers) const
{
    for (auto & controller : m_controllers) {
        controllers.push_back(controller->m_name);
    }
}


void mtsPSMove::state_command(const std::string & command)
{
    std::string humanReadableMessage;
    prmOperatingState::StateType newOperatingState;
    try {
        if (m_operating_state.ValidCommand(prmOperatingState::CommandTypeFromString(command),
                                           newOperatingState, humanReadableMessage)) {
            if (command == "enable") {
                m_operating_state.SetState(prmOperatingState::ENABLED);
            } else if (command == "disable") {
                m_operating_state.SetState(prmOperatingState::DISABLED);
            } else if (command == "home") {
                m_operating_state.SetIsHomed(true);
            } else if (command == "unhome") {
                m_operating_state.SetIsHomed(false);
            } else if (command == "pause") {
            } else if (command == "resume") {
            }
        } else {
            m_interface->SendWarning(this->GetName() + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        m_interface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
    dispatch_operating_state();
}


void mtsPSMove::dispatch_operating_state(void)
{
    // for now, consider operating state is for the whole system
    m_operating_state_event(m_operating_state);
    for (auto controller : m_controllers) {
        controller->m_operating_state_event(m_operating_state);
    }
}


void mtsPSMove::update_data(void)
{
    // update orientation, buttons, trigger, battery
    for (auto controller : m_controllers) {
        controller->update_data();
    }

    // Drive camera state machine
    const double now = osaGetTime();
    camera_step(now);

    if (m_tracker_handle &&
        (m_camera_status == CameraStatus::Calibrating ||
         m_camera_status == CameraStatus::Ready)) {
        psmove_tracker_update_image(m_tracker_handle);
        psmove_tracker_update(m_tracker_handle, nullptr);
    }

    // update position if we have a tracker and fusion
    if (m_tracker_handle && m_fusion_handle) {
        for (auto controller : m_controllers) {
            auto& move = controller->m_move_handle;

            float x_cm = 0.f, y_cm = 0.f, z_cm = 0.f;
            psmove_fusion_get_position(m_fusion_handle, move, &x_cm, &y_cm, &z_cm);
            const bool ok = std::isfinite(x_cm) && std::isfinite(y_cm) && std::isfinite(z_cm);
            if (ok) {
                const double X = 0.01 * static_cast<double>(x_cm); // cm -> m
                const double Y = 0.01 * static_cast<double>(y_cm); // cm -> m
                const double Z = 0.01 * static_cast<double>(z_cm); // cm -> m
                controller->m_measured_cp_local.Position().Translation().Assign(X, Y, Z);
            }
            // If not ok, we keep last translation to avoid output flicker on intermittent tracking.
        }
    } else {
        // No tracker or fusion - Do nothing
    }

    // update all poses and point cloud after base frame transform and filtering (if enabled)
    size_t index = 0;
    for (auto controller : m_controllers) {
        auto & c_measured_cp_local = controller->m_measured_cp_local.Position();
        auto & c_measured_cp = controller->m_measured_cp.Position();
        vctFrm3 _pose;
        m_base_frame.ApplyTo(c_measured_cp_local, _pose);

        // apply filters
        if (controller->m_filters_enabled) {
            auto &T_new = _pose.Translation();
            auto &T_prev = c_measured_cp.Translation();
            double Xout = T_new.X(), Yout = T_new.Y(), Zout = T_new.Z();
            // apply tiny deadband around last published value to eliminate shimmer
            const double db = controller->m_deadband_m;
            if (std::fabs(T_new.X() - T_prev.X()) < db) { Xout = T_prev.X(); }
            else { Xout = controller->m_filter_x.filter(T_new.X(), now); }
            if (std::fabs(T_new.Y() - T_prev.Y()) < db) { Yout = T_prev.Y(); }
            else { Yout = controller->m_filter_y.filter(T_new.Y(), now); }
            if (std::fabs(T_new.Z() - T_prev.Z()) < db) { Zout = T_prev.Z(); }
            else { Zout = controller->m_filter_z.filter(T_new.Z(), now); }
            _pose.Translation().Assign(Xout, Yout, Zout);
        }

        c_measured_cp.FromNormalized(_pose);
        // "point cloud"
        m_measured_cp_array.Positions().at(index) = _pose;
        ++index;
    }

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


void mtsPSMove::camera_start_if_needed(void)
{
    if (m_tracker_handle) return;

    const double now = osaGetTime();
    if ((now - m_cam_last_enable_try_sec) < m_cam_retry_period_sec) {
        return;
    }

    camera_set_status(CameraStatus::Starting);
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
    for (const auto controller : m_controllers) {
        psmove_set_rate_limiting(controller->m_move_handle, 0);
    }

    // Kick calibration on next step
}


void mtsPSMove::camera_step(double now_sec)
{
    // Honor desired state
    if (!m_camera_requested) {
        if (m_tracker_handle) {
            camera_stop();
        }
        return;
    }
    if (!m_tracker_handle) {
        camera_start_if_needed();
        return;
    }

    // Starting -> issue enable; if instant CALIBRATED, become Ready; else go Calibrating
    if (m_camera_status == CameraStatus::Starting) {
        bool ready = true;
        for (const auto controller : m_controllers) {
            auto st = psmove_tracker_enable(m_tracker_handle, controller->m_move_handle);
            ready &= (st == Tracker_CALIBRATED);
        }
        if (ready) {
            camera_set_status(CameraStatus::Ready);
            // Initialize fusion now that tracker is ready
            init_fusion();
            // >>> re-enable rate limiting once Ready <<<
            for (const auto controller : m_controllers) {
                psmove_set_rate_limiting(controller->m_move_handle, 1);
            }
            return;
        }
        m_cam_calib_start_sec = now_sec;
        camera_set_status(CameraStatus::Calibrating);
        m_interface->SendStatus("Camera: calibrating");
        return;
    }

    // Calibrating -> wait until CALIBRATED or timeout; on timeout recreate later
    if (m_camera_status == CameraStatus::Calibrating) {
        bool calibrated = true;
        for (const auto controller : m_controllers) {
            auto st = psmove_tracker_enable(m_tracker_handle, controller->m_move_handle);
            calibrated &= (st == Tracker_CALIBRATED);
        }
        if (calibrated) {
            // write a message that we are calibrated
            m_interface->SendStatus("Camera: calibrated");
            camera_set_status(CameraStatus::Ready);
            // Initialize fusion now that tracker is ready
            init_fusion();
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
    if (m_fusion_handle) {
        psmove_fusion_free(m_fusion_handle);
        m_fusion_handle = nullptr;
    }
    // Re-enable rate limiting when stopping camera
    for (const auto controller : m_controllers) {
        psmove_set_rate_limiting(controller->m_move_handle, 1);
    }
    camera_set_status(CameraStatus::Disabled);
    m_cam_last_enable_try_sec = 0.0;
    m_cam_calib_start_sec = 0.0;
}


// call once after tracker is ready
void mtsPSMove::init_fusion()
{
    // choose a sensible clip range around typical PS Move distances (in cm).
    // e.g., 10 cm to 500 cm. Adjust for your setup/room.
    const float z_near_cm = 10.f;
    const float z_far_cm  = 500.f;
    m_fusion_handle = psmove_fusion_new(m_tracker_handle, z_near_cm, z_far_cm);
}



