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

#pragma once

#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstVector/vctQuaternionRotation3.h>
#include <cisstVector/vctFrame4x4.h>
#include <cisstVector/vctTransformationTypes.h>
#include <cisstCommon/cmnUnits.h>

#include <sawPSMove/sawPSMoveExport.h>

extern "C" {
#include <psmove.h>
#ifdef SAW_PSMOVE_HAVE_TRACKER
#include <psmove_tracker.h>
#endif
}

class CISST_EXPORT mtsPSMove : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsPSMove(const std::string & name, double periodInSeconds = 0.01);
    ~mtsPSMove() override;

    void Configure(const std::string & configFile = "") override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

private:
    // State: measured pose (CRTK "measured_cp")
    prmPositionCartesianGet M_measured_cp;

    // PSMove handles
    PSMove * Move{nullptr};
#ifdef SAW_PSMOVE_HAVE_TRACKER
    PSMoveTracker * Tracker{nullptr};
#endif

    void UpdatePoseFromPSMove();
};
CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMove);

