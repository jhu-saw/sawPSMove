/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawPSMove/mtsPSMove.h>

#include <cisstCommon/cmnLogger.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsComponentManager.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsPSMove, mtsTaskContinuous);

mtsPSMove::mtsPSMove(const std::string & name, double periodInSeconds)
: mtsTaskContinuous(name, periodInSeconds)
{
    StateTable.AddData(M_measured_cp, "measured_cp");

    auto * provided = this->AddInterfaceProvided("Controller");
    if (provided) {
        provided->AddCommandReadState(StateTable, M_measured_cp, "measured_cp");
        provided->AddCommandReadState(StateTable, StateTable.PeriodStats, "GetPeriodStatistics");
    }
}

mtsPSMove::~mtsPSMove()
{
    // Clean in Cleanup()
}

void mtsPSMove::Configure(const std::string &)
{
    // No external JSON; default identity pose until first valid read
}

void mtsPSMove::Startup(void)
{
    Move = psmove_connect();
    if (!Move) {
        CMN_LOG_CLASS_INIT_ERROR << "PSMove connect failed" << std::endl;
        M_measured_cp.SetValid(false);
        return;
    }
    psmove_enable_orientation(Move, PSMove_True);

#ifdef SAW_PSMOVE_HAVE_TRACKER
    Tracker = psmove_tracker_new();
    if (Tracker) {
        // Calibrate tracker for this controller
        while (psmove_tracker_enable(Tracker, Move) != Tracker_CALIBRATED) { /* retry */ }
    }
#endif

    vctFrm3 I; I.Identity();
    M_measured_cp.SetPosition(I);
    M_measured_cp.SetValid(false);

    CMN_LOG_CLASS_INIT_VERBOSE << "PSMove started" << std::endl;
}

void mtsPSMove::Run(void)
{
    ProcessQueuedCommands();

    if (!Move) {
        M_measured_cp.SetValid(false);
        osaSleep(0.01);
        return;
    }

    // Process controller events; allow orientation reset on Move button
    while (psmove_poll(Move)) {
        if (psmove_get_buttons(Move) & Btn_MOVE) {
            psmove_reset_orientation(Move);
        }
    }

    UpdatePoseFromPSMove();
    M_measured_cp.SetTimestamp(mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime());
}

void mtsPSMove::Cleanup(void)
{
#ifdef SAW_PSMOVE_HAVE_TRACKER
    if (Tracker) { psmove_tracker_free(Tracker); Tracker = nullptr; }
#endif
    if (Move) { psmove_disconnect(Move); Move = nullptr; }
}

void mtsPSMove::UpdatePoseFromPSMove()
{
    float q0=0, q1=0, q2=0, q3=0;
    psmove_get_orientation(Move, &q0, &q1, &q2, &q3); // returns [w,x,y,z]

    const double n2 = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (n2 < 1e-8) {
        M_measured_cp.SetValid(false);
        return;
    }

    // Quaternion -> rotation
    vctQuatRot3 qcisst(q0, q1, q2, q3);   // (w,x,y,z)
    vctRot3 R(qcisst);

    // Position proxy (0 if tracker unavailable)
    vct3 p(0.0);
#ifdef SAW_PSMOVE_HAVE_TRACKER
    if (Tracker) {
        psmove_tracker_update_image(Tracker);
        psmove_tracker_update(Tracker, nullptr);
        if (psmove_tracker_get_status(Tracker, Move) == Tracker_TRACKING) {
            float x, y, radius;
            psmove_tracker_get_position(Tracker, Move, &x, &y, &radius);
            // Normalize to a crude workspace centered at 0
            p.X() =  (1.0 - (x / 640.0) * 2.0);   // [-1,1]
            p.Y() =  (1.0 - (y / 480.0) * 2.0);   // [-1,1]
            p.Z() =   1.0 - (radius / 150.0);     // approx depth
        }
    }
#endif

    vctFrm3 T(R, p);
    M_measured_cp.SetPosition(T);
    M_measured_cp.SetValid(true);
}

