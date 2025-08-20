#
#
# (C) Copyright 2013-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---


set(REQUIRED_CISST_LIBRARIES
    cisstCommon
    cisstVector
    cisstCommonQt
    cisstVectorQt
    cisstOSAbstraction
    cisstMultiTask
    cisstMultiTaskQt
    cisstParameterTypes
    cisstQt
)

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
if (cisst_FOUND_AS_REQUIRED)
  include(${CISST_USE_FILE})

  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  set(CMAKE_AUTOMOC ON)

  add_library(sawPSMoveQt
      ${sawPSMove_HEADER_DIR}/sawPSMoveQtExport.h
      ${sawPSMove_HEADER_DIR}/mtsPSMoveQtWidget.h
      mtsPSMoveQtWidget.cpp)

  set_property(TARGET sawPSMoveQt PROPERTY FOLDER "sawPSMove")
  cisst_target_link_libraries(sawPSMoveQt ${REQUIRED_CISST_LIBRARIES})

  # Export Qt lib to parent so examples/ROS can link it if desired
  set(sawPSMoveQt_LIBRARIES sawPSMoveQt PARENT_SCOPE)
else()
  message("Information: Qt widget requires cisst Qt components.")
endif()