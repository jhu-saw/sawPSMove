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
project (sawPSMove VERSION 0.1.0)

# Core libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.5 REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# catkin/ROS paths
cisst_set_output_path ()

set (SAW_PSMOVEAPI_PROVIDER "BUNDLED" CACHE STRING
     "PSMoveAPI provider (BUNDLED or SYSTEM)")
set_property (CACHE SAW_PSMOVEAPI_PROVIDER PROPERTY STRINGS BUNDLED SYSTEM)
string (TOUPPER "${SAW_PSMOVEAPI_PROVIDER}" SAW_PSMOVEAPI_PROVIDER)

if (SAW_PSMOVEAPI_PROVIDER STREQUAL "BUNDLED")
  include (ExternalProject)

  set (SAW_PSMOVEAPI_GIT_TAG
       "256b6299e6076fe7b113e7277ed67016903f067a"
       CACHE STRING "PSMoveAPI Git commit or tag")
  option (SAW_PSMOVEAPI_UPDATE
          "Allow the bundled PSMoveAPI checkout to contact its Git remote"
          OFF)
  if (SAW_PSMOVEAPI_UPDATE)
    set (_psmoveapi_update_disconnected FALSE)
  else ()
    set (_psmoveapi_update_disconnected TRUE)
  endif ()
  set (_psmoveapi_library
       "${LIBRARY_OUTPUT_PATH}/${CMAKE_SHARED_LIBRARY_PREFIX}psmoveapi${CMAKE_SHARED_LIBRARY_SUFFIX}")
  set (_psmoveapi_tracker_library
       "${LIBRARY_OUTPUT_PATH}/${CMAKE_SHARED_LIBRARY_PREFIX}psmoveapi_tracker${CMAKE_SHARED_LIBRARY_SUFFIX}")
  set (_psmoveapi_utility
       "${EXECUTABLE_OUTPUT_PATH}/psmove${CMAKE_EXECUTABLE_SUFFIX}")
  if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.28)
    set (_psmoveapi_job_server BUILD_JOB_SERVER_AWARE TRUE)
  endif ()

  ExternalProject_Add (sawPSMove-psmoveapi
    GIT_REPOSITORY https://github.com/thp/psmoveapi.git
    GIT_TAG ${SAW_PSMOVEAPI_GIT_TAG}
    GIT_SUBMODULES external/PS3EYEDriver external/hidapi external/libusb-1.0
    UPDATE_DISCONNECTED ${_psmoveapi_update_disconnected}
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}/psmoveapi"
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
      -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON
      -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${LIBRARY_OUTPUT_PATH}
      -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=${EXECUTABLE_OUTPUT_PATH}
      -DPSMOVEAPI_VENDORING=ON
      -DPSMOVE_BUILD_TRACKER=ON
      -DPSMOVE_USE_PS3EYE_DRIVER=OFF
      -DPSMOVE_USE_SIXPAIR=OFF
    BUILD_COMMAND
      ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG>
      --target psmoveapi psmoveapi_tracker psmove
    ${_psmoveapi_job_server}
    BUILD_BYPRODUCTS
      ${_psmoveapi_library}
      ${_psmoveapi_tracker_library}
      ${_psmoveapi_utility}
    INSTALL_COMMAND ""
    )

  ExternalProject_Get_Property (sawPSMove-psmoveapi SOURCE_DIR BINARY_DIR)
  set (PSMoveAPI_INCLUDE_DIRS "${SOURCE_DIR}/include" "${BINARY_DIR}")

  add_library (PSMoveAPI::Core SHARED IMPORTED GLOBAL)
  set_target_properties (PSMoveAPI::Core PROPERTIES
                         IMPORTED_LOCATION "${_psmoveapi_library}")
  add_dependencies (PSMoveAPI::Core sawPSMove-psmoveapi)

  add_library (PSMoveAPI::Tracker SHARED IMPORTED GLOBAL)
  set_target_properties (PSMoveAPI::Tracker PROPERTIES
                         IMPORTED_LOCATION "${_psmoveapi_tracker_library}")
  add_dependencies (PSMoveAPI::Tracker sawPSMove-psmoveapi)

  set (PSMoveAPI_LIBRARIES PSMoveAPI::Core PSMoveAPI::Tracker)
  message (STATUS "PSMoveAPI provider: bundled (${SAW_PSMOVEAPI_GIT_TAG})")

elseif (SAW_PSMOVEAPI_PROVIDER STREQUAL "SYSTEM")
  find_package (PkgConfig REQUIRED)
  pkg_check_modules (PSMoveAPI REQUIRED IMPORTED_TARGET psmoveapi)
  find_library (PSMoveAPI_TRACKER_LIBRARY
                NAMES psmoveapi_tracker
                HINTS ${PSMoveAPI_LIBRARY_DIRS}
                NO_DEFAULT_PATH
                REQUIRED)
  set (PSMoveAPI_LIBRARIES PkgConfig::PSMoveAPI ${PSMoveAPI_TRACKER_LIBRARY})
  message (STATUS "PSMoveAPI provider: system (${PSMoveAPI_VERSION})")

else ()
  message (FATAL_ERROR
           "SAW_PSMOVEAPI_PROVIDER must be BUNDLED or SYSTEM, not '${SAW_PSMOVEAPI_PROVIDER}'")
endif ()

# create/configure file for find_package (sawPSMove)
set (sawPSMove_INCLUDE_DIR
     "${sawPSMove_SOURCE_DIR}/include"
     "${sawPSMove_BINARY_DIR}/include")
set (sawPSMove_HEADER_DIR "${sawPSMove_SOURCE_DIR}/include/sawPSMove")
set (sawPSMove_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
set (sawPSMove_LIBRARIES sawPSMove)

include_directories (${sawPSMove_INCLUDE_DIR} ${PSMoveAPI_INCLUDE_DIRS})

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

target_link_libraries (sawPSMove
                       PUBLIC ${cisst_LIBRARIES}
                       PRIVATE ${PSMoveAPI_LIBRARIES})
if (SAW_PSMOVEAPI_PROVIDER STREQUAL "BUNDLED")
  add_dependencies (sawPSMove sawPSMove-psmoveapi)
  set_target_properties (sawPSMove PROPERTIES INSTALL_RPATH "$ORIGIN")
endif ()

# Install target for headers and library
install (DIRECTORY "${sawPSMove_SOURCE_DIR}/include/sawPSMove"
                   "${sawPSMove_BINARY_DIR}/include/sawPSMove"
         DESTINATION include
         COMPONENT sawPSMove-dev)

install (TARGETS sawPSMove COMPONENT sawPSMove
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

if (SAW_PSMOVEAPI_PROVIDER STREQUAL "BUNDLED")
  install (FILES ${_psmoveapi_library} ${_psmoveapi_tracker_library}
           DESTINATION lib
           COMPONENT sawPSMove)
  install (PROGRAMS ${_psmoveapi_utility}
           DESTINATION bin
           COMPONENT sawPSMove)
endif ()
