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
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawPSMove/sawPSMoveExport.h>

// Forward declare to avoid leaking the C header in users’ includes
struct _PSMove;
typedef struct _PSMove PSMove;

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

    // Utility commands
    void set_LED(const vctDouble3 &rgb);    // 0..1
    void rumble(const double & strength); // 0..1
    void reset_orientation(void);

protected:

    // Internals
    void initialize(void);
    void update_data(void);

    // PSMove handle
    PSMove * m_move_handle = nullptr;
    int m_controller_index = 0;
    bool m_orientation_available = false;

    // State
    prmPositionCartesianGet m_measured_cp;    // Pose
    prmStateJoint m_gripper_measured_js;
    vctDouble3 m_accel;                       // accel raw
    vctDouble3 m_gyro;                        // gyro raw
    unsigned int m_buttons{0};                // button mask
    double m_trigger{0.0};                    // 0..1
    int m_battery{0};                         // 0..100, 99 for charging (approx)

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
