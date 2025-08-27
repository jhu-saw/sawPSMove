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

#ifndef _mtsPSMove_h
#define _mtsPSMove_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawPSMove/sawPSMoveExport.h>

// Forward declare to avoid leaking the C header in users’ includes
struct _PSMove;
typedef struct _PSMove PSMove;

struct _PSMoveTracker;
typedef struct _PSMoveTracker PSMoveTracker;

class CISST_EXPORT mtsPSMove: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    // Default: discover controller 0
    explicit mtsPSMove(const std::string & component_name, const double & period_in_seconds = 0.03);

    inline mtsPSMove(const mtsTaskPeriodicConstructorArg & arg):
        mtsTaskPeriodic(arg) {
        initialize();
    }

    // Select controller by numeric/string hint (e.g. "id:1" or "1")
    mtsPSMove(const std::string & componentName, const std::string & connectionHint);

    ~mtsPSMove() override;

    // cisst Task interface
    void Configure(const std::string &args) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

protected:

    // Utility commands
    void set_LED(const vctDouble3 &rgb);    // 0..1
    void rumble(const double & strength); // 0..1
    void reset_orientation(void);

    // Internals
    void initialize(void);
    void update_data(void);
    void state_command(const std::string & command);

    // Camera commands
    void enable_camera(const bool &enable); // This enables the camera. Might need a better name.
    void calibrate_camera(void); // Try to calibrate/enable tracking; might need a better name.
    void set_intrinsics(const vctDouble4 &fx_fy_cx_cy);
    void set_sphere_radius(const double &radius_m);
    void set_camera_translation(const vctDouble3 &translation);
    void set_camera_rotation(const vctMatRot3 &rotation);

    // PSMove handle
    PSMove * m_move_handle = nullptr;
    // Tracker handle (optional)
    PSMoveTracker * m_tracker_handle = nullptr;

    int m_controller_index = 0;
    bool m_orientation_available = false;

    // Camera pipeline configuration
    bool   m_camera_enabled = true; // FIXME: default to true for now. 
    bool   m_camera_calibrated = false;
    double m_fx = 800.0, m_fy = 800.0, m_cx = 320.0, m_cy = 240.0;  // pixels
    double m_sphere_radius_m = 0.0225;                               // ~22.5 mm ball

    // Camera to world transform
    vctMatRot3 m_R_world_cam;  // 3x3 rotation
    vctDouble3 m_t_world_cam;  // world ⟵ cam

    // State
    prmOperatingState m_operating_state;
    prmPositionCartesianGet m_measured_cp;    // Pose
    prmStateJoint m_gripper_measured_js;
    vctDouble3 m_accel;                       // accel raw
    vctDouble3 m_gyro;                        // gyro raw
    unsigned int m_buttons{0};                // button mask
    double m_trigger{0.0};                    // 0..1
    int m_battery{0};                         // 0..100, 99 for charging (approx)

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

    // Provided interface
    mtsInterfaceProvided * m_interface = nullptr;

    struct {
        prmEventButton pressed;
        prmEventButton released;
    } m_event_payloads;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMove);

#endif // _mtsPSMove_h
