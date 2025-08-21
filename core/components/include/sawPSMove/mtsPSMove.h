// sawPSMove/components/include/sawPSMove/mtsPSMove.h
#pragma once

#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstVector/vctMatrixRotation3.h>
#include <cisstVector/vctFrame4x4.h>

#include <sawPSMove/sawPSMoveExport.h>

// Forward declare to avoid leaking the C header in users’ includes
struct _PSMove;
typedef struct _PSMove PSMove;

class SAW_PSMOVE_EXPORT mtsPSMove : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    // Default: discover controller 0
    explicit mtsPSMove(const std::string & componentName);
    // Select controller by numeric/string hint (e.g. "id:1" or "1")
    mtsPSMove(const std::string & componentName, const std::string & connectionHint);

    ~mtsPSMove() override;

    // cisst Task interface
    void Configure(const std::string &args) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

    // Utility commands
    void SetLED(const vctDouble3 &rgb);    // 0..1
    void SetRumble(const double & strength); // 0..1
    void ResetOrientation(void);

protected:
    // PSMove handle
    PSMove *Move{nullptr};
    int ControllerIndex{0};
    bool OrientationAvailable{false};

    // State
    prmPositionCartesianGet m_measured_cp;    // Pose
    vctDouble3 m_accel;                       // accel raw
    vctDouble3 m_gyro;                        // gyro raw
    unsigned int m_buttons{0};                // button mask
    double m_trigger{0.0};                    // 0..1
    double m_battery{0.0};                    // 0..1 (approx)

    // State table
    mtsStateTable *Table{nullptr};

    // Provided interface
    mtsInterfaceProvided *Interface{nullptr};

    // Internals
    void UpdateFromController();
    void QuaternionToRotation(double w, double x, double y, double z, vctMatRot3 &R) const;
    static double Clamp01(double v) { return (v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v)); }
};
CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMove)
