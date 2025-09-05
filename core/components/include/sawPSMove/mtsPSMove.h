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
struct _PSMoveTracker;
typedef struct _PSMoveTracker PSMoveTracker;

class mtsPSMoveController;

class CISST_EXPORT mtsPSMove: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

    friend class mtsPSMoveController;

public:

    // Default: discover controller 0
    explicit mtsPSMove(const std::string & component_name, const double & period_in_seconds = 0.001);

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

    void get_controller_names(std::list<std::string> & controllers) const;

    enum class CameraStatus {
        Disabled,
        Starting,       // tracker handle being created
        Calibrating,    // enable() issued, waiting on Tracker_CALIBRATED
        Ready,          // good pose updates
        Error           // unrecoverable error
    };

protected:

    // Camera control API
    void enable_camera(const bool &enable);
    void set_intrinsics(const vctDouble4 &fx_fy_cx_cy);
    void set_camera_translation(const vctDouble3 &translation);
    void set_camera_rotation(const vctMatRot3 &rotation);

    // Internals
    void initialize(void);
    void update_data(void);
    void state_command(const std::string & command);

    // ---- Camera state machine (internal) ----
    void camera_start_if_needed(void);
    void camera_step(const double now_sec);              // drive calibration / status
    void camera_stop(void);
    void camera_set_status(const CameraStatus s);
    void camera_init_image_center_from_tracker(void);
    void camera_init_fx_fy_if_needed(const int width_px, const int height_px);


    // Tracker handle (optional)
    PSMoveTracker * m_tracker_handle = nullptr;

    // Camera config
    bool m_camera_requested = false;               // user's desired state
    CameraStatus m_camera_status = CameraStatus::Disabled;
    bool m_camera_have_pose = false;
    double m_fx = 0.0, m_fy = 0.0, m_cx = 0.0, m_cy = 0.0; // intrinsics
    vctMatRot3 m_R_world_cam;                      // camera rotation in world
    vctDouble3 m_t_world_cam;                      // camera translation in world
    double m_cam_last_enable_try_sec = 0.0;
    double m_cam_calib_start_sec = 0.0;
    double m_cam_retry_period_sec = 1.0;           // re-create tracker / re-enable cadence
    double m_cam_calib_timeout_sec = 15.0;          // time to reach CALIBRATED

    // State
    prmOperatingState m_operating_state;
    mtsFunctionWrite m_operating_state_event;

    // Provided interface
    mtsInterfaceProvided * m_interface = nullptr;

    // all controllers
    std::list<mtsPSMoveController *> m_controllers;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMove);

#endif // _mtsPSMove_h
