/* -*- Mode: C++; c-basic-offset: 4 -*- */
#include <cisstCommon/cmnLogger.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsComponentManager.h>
#include <sawPSMove/mtsPSMove.h>

int main(int, char **)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    auto * dev = new mtsPSMove("PSMove", 0.01); // 100 Hz
    auto * mgr = mtsComponentManager::GetInstance();
    mgr->AddComponent(dev);

    mgr->CreateAllAndWait(2.0 * cmn_s);
    mgr->StartAllAndWait(2.0 * cmn_s);

    for (int i = 0; i < 500; ++i) {
        prmPositionCartesianGet cp =
            dev->StateTable.GetItem<prmPositionCartesianGet>("measured_cp");
        if (cp.Valid()) {
            const auto & F = cp.Position();
            std::cout << "p = [" << F.Translation() << "]\n";
        }
        osaSleep(0.02);
    }

    mgr->KillAllAndWait(2.0 * cmn_s);
    mgr->Cleanup();
    cmnLogger::Kill();
    return 0;
}
