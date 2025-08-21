/* -*- Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*- */
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>

#include <sawPSMove/mtsPSMove.h>
#include <sawPSMove/mtsPSMoveQtWidget.h>

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

#include <QApplication>

int main(int argc, char * argv[])
{
    // Logging
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // ROS node
    cisst_ral::ral ral(argc, argv, "psmove");
    auto rosNode = ral.node();

    // CLI
    cmnCommandLineOptions options;
    std::string connectionHint = "";     // e.g., "id:1" or "1"
    std::list<std::string> managerCfg;
    double rosPeriod = 10.0 * cmn_ms;
    bool textOnly = false;
    bool darkMode = false;

    options.AddOptionOneValue("c", "connection",
                              "PSMove connection hint (index: 'id:N' or 'N')",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &connectionHint);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerCfg);
    options.AddOptionOneValue("p", "ros-period",
                              "bridge period in seconds (default 0.01s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionNoValue("t", "text-only", "no Qt widgets");
    options.AddOptionNoValue("D", "dark-mode", "Qt dark palette");
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }
    textOnly = options.IsSet("text-only");
    darkMode = options.IsSet("dark-mode");

    // SAW component
    mtsPSMove * device = connectionHint.empty()
        ? new mtsPSMove("PSMove")
        : new mtsPSMove("PSMove", connectionHint);

    auto * cm = mtsComponentManager::GetInstance();
    cm->AddComponent(device);

    // CRTK ROS bridge (namespace "psmove")
    auto * bridge = new mts_ros_crtk_bridge_provided("psmove_crtk_bridge", rosNode);
    cm->AddComponent(bridge);

    bridge->bridge_interface_provided(device->GetName(),
                                      "Controller",
                                      "psmove",
                                      rosPeriod);
    bridge->Connect();

    // Optional Qt UI
    QApplication * app = nullptr;
    mtsPSMoveQtWidget * widget = nullptr;
    if (!textOnly) {
        app = new QApplication(argc, argv);
        cmnQt::QApplicationExitsOnCtrlC();
        if (darkMode) cmnQt::SetDarkMode();
        widget = new mtsPSMoveQtWidget("PSMove-GUI");
        cm->AddComponent(widget);
        cm->Connect(widget->GetName(), "Device",
                    device->GetName(), "Controller");
    }

    if (!cm->ConfigureJSON(managerCfg)) {
        CMN_LOG_INIT_ERROR << "Failed to configure component-manager (see cisstLog)" << std::endl;
        return -1;
    }

    cm->CreateAllAndWait(5.0 * cmn_s);
    cm->StartAllAndWait(5.0 * cmn_s);

    if (widget) { widget->show(); app->exec(); }
    else {
        std::cout << "Press 'q' to quit" << std::endl;
        while (cmnGetChar() != 'q') { /* spin */ }
    }

    cisst_ral::shutdown();
    cm->KillAllAndWait(5.0 * cmn_s);
    cm->Cleanup();
    cmnLogger::Kill();
    return 0;
}
