#
# (C) Copyright 2016-2019 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project(sawPSMove)

# cisst
set(REQUIRED_CISST_LIBRARIES
    cisstCommon
    cisstVector
    cisstNumerical
    cisstOSAbstraction
    cisstMultiTask
    cisstParameterTypes
)
find_package(cisst 1.0.11 COMPONENTS ${REQUIRED_CISST_LIBRARIES} REQUIRED)
include(${CISST_USE_FILE})

# PSMove API (headers & libs should be visible via CMAKE_PREFIX_PATH or pkg-config)
find_path(PSMOVE_INCLUDE_DIR psmove.h)
find_library(PSMOVE_LIB psmoveapi)
# Optional tracker + OpenCV (for camera pose proxy)
find_library(PSMOVE_TRACKER_LIB psmoveapi_tracker)
find_package(OpenCV QUIET COMPONENTS core imgproc highgui)

if (PSMOVE_TRACKER_LIB AND OpenCV_FOUND)
  add_compile_definitions(SAW_PSMOVE_HAVE_TRACKER)
endif()

# Headers & sources
set(sawPSMove_HEADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/sawPSMove")
include_directories(
  ${sawPSMove_HEADER_DIR}
  ${PSMOVE_INCLUDE_DIR}
)

add_library(sawPSMove
  code/mtsPSMove.cpp
  ${sawPSMove_HEADER_DIR}/sawPSMoveExport.h
  ${sawPSMove_HEADER_DIR}/mtsPSMove.h
)

# Public include dir so downstream targets can include <sawPSMove/...>
target_include_directories(sawPSMove PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link cisst
cisst_target_link_libraries(sawPSMove ${REQUIRED_CISST_LIBRARIES})

# Link PSMove core
target_link_libraries(sawPSMove ${PSMOVE_LIB})

# Link tracker + OpenCV if present
if (PSMOVE_TRACKER_LIB AND OpenCV_FOUND)
  target_link_libraries(sawPSMove ${PSMOVE_TRACKER_LIB} ${OpenCV_LIBS})
endif()

set_target_properties(sawPSMove PROPERTIES FOLDER "sawPSMove")

# Qt widget subdir
if (CISST_HAS_QT)
  add_subdirectory(code/Qt)
endif()

# Install
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/sawPSMove"
        DESTINATION include)
install(TARGETS sawPSMove
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
