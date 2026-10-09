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

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt)

# cisst
find_package (cisst 1.5 REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# for Qt generated header files
include_directories (${CMAKE_CURRENT_BINARY_DIR})
set (CMAKE_AUTOMOC ON)

add_library (sawPSMoveQt
             ${sawPSMove_HEADER_DIR}/mtsPSMoveQtWidget.h
             mtsPSMoveQtWidget.cpp
             ${SAW_FORCE_DIMENSION_SDK_QT_WRAP_CPP})
set_target_properties (sawPSMoveQt PROPERTIES
                       VERSION ${sawPSMove_VERSION}
                       FOLDER "sawPSMove")
target_link_libraries (sawPSMoveQt
                       PUBLIC sawPSMove
                              ${cisst_LIBRARIES})

install (TARGETS sawPSMoveQt COMPONENT sawPSMove-Qt
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
