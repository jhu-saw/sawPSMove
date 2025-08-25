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

cmake_minimum_required (VERSION 3.10)
project (sawPSMove VERSION 0.1.0)

# Core libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.2.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawPSMove)
  set (sawPSMove_INCLUDE_DIR
       "${sawPSMove_SOURCE_DIR}/include"
       "${sawPSMove_BINARY_DIR}/include")
  set (sawPSMove_HEADER_DIR "${sawPSMove_SOURCE_DIR}/include/sawPSMove")
  set (sawPSMove_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawPSMove_LIBRARIES sawPSMove psmoveapi)

  include_directories (${sawPSMove_INCLUDE_DIR})

  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawPSMove_LIBRARIES ${sawPSMove_LIBRARIES} sawPSMoveQt)
  endif ()

  # add all config files for this component
  cisst_add_config_files (sawPSMove)

  add_library (sawPSMove
               code/mtsPSMove.cpp
               ${sawPSMove_HEADER_DIR}/sawPSMoveExport.h
               ${sawPSMove_HEADER_DIR}/mtsPSMove.h
               ${sawPSMove_BINARY_DIR}/include/sawPSMove/sawPSMoveRevision.h)

  set_target_properties (sawPSMove PROPERTIES
                         VERSION ${sawPSMove_VERSION}
                         FOLDER "sawPSMove")

  target_link_libraries (sawPSMove psmoveapi)

  cisst_target_link_libraries (sawPSMove ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY "${sawPSMove_SOURCE_DIR}/include/sawPSMove"
                     "${sawPSMove_BINARY_DIR}/include/sawPSMove"
           DESTINATION include
           COMPONENT sawPSMove-dev)

  install (TARGETS sawPSMove COMPONENT sawPSMove
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
