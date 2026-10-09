#
# (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
if (POLICY CMP0144)
  cmake_policy (SET CMP0144 NEW)
endif ()
project (sawPSMoveExamples VERSION 2.1.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonQt
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.5 REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# catkin/ROS paths
cisst_set_output_path ()

  # sawPSMove has been compiled within cisst, we should find it automatically
find_package (sawPSMove 0.1.0 REQUIRED)

if (CISST_HAS_QT)

    # sawPSMove configuration
    include_directories (${sawPSMove_INCLUDE_DIR})
    link_directories (${sawPSMove_LIBRARY_DIR})

    add_executable (sawPSMoveQtExample main.cpp)
    set_target_properties (sawPSMoveQtExample PROPERTIES
                           COMPONENTS sawPSMove-Examples
                           FOLDER "sawPSMove")

    # link against non cisst libraries and saw components
    target_link_libraries (sawPSMoveQtExample
                           ${sawPSMove_LIBRARIES}
                           ${sawPSMoveQt_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    target_link_libraries (sawPSMoveQtExample ${cisst_LIBRARIES})

    install (TARGETS sawPSMoveQtExample COMPONENT sawPSMove-Examples
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

else ()
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires cisst Qt libraries")
endif ()
