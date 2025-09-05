/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2025-08-21

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawPSMove/mtsPSMove.h>
#include <sawPSMove/mtsPSMoveQtWidget.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsPSMove", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    std::string json_config_file = "";
    std::list<std::string> manager_config;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &json_config_file);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &manager_config);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    // check that all required options have been provided
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsPSMove * ps_move = new mtsPSMove("PSMove");
    ps_move->Configure(json_config_file);

    // add the components to the component manager
    mtsManagerLocal * component_manager = mtsComponentManager::GetInstance();
    component_manager->AddComponent(ps_move);

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }
        
    // organize all widgets in a tab widget
    QTabWidget * tab_widget = new QTabWidget;
    mtsPSMoveQtWidget * controller_widget;

    // Qt Widget(s)
    std::list<std::string> controllers;
    ps_move->get_controller_names(controllers);
    for (const auto & controller : controllers) {
        controller_widget = new mtsPSMoveQtWidget(controller + "-gui");
        component_manager->AddComponent(controller_widget);
        component_manager->Connect(controller_widget->GetName(), "controller",
                                   ps_move->GetName(), controller);
        tab_widget->addTab(controller_widget, controller.c_str());
    }

    // custom user components
    if (!component_manager->ConfigureJSON(manager_config)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    component_manager->CreateAllAndWait(5.0 * cmn_s);
    component_manager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tab_widget->show();
    application.exec();

    // stop all logs
    cmnLogger::Kill();

    // kill all components and perform cleanup
    component_manager->KillAllAndWait(5.0 * cmn_s);
    component_manager->Cleanup();

    return 0;
}
