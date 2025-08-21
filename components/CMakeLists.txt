# cmake_minimum_required(VERSION 3.10)
# project(sawPSMove VERSION 0.1.0)

# # cisst libs (same set as Optoforce)
# set(REQUIRED_CISST_LIBRARIES
#   cisstCommon
#   cisstVector
#   cisstNumerical
#   cisstOSAbstraction
#   cisstMultiTask
#   cisstParameterTypes)

# find_package(cisst 1.2.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# if(cisst_FOUND_AS_REQUIRED)
#   include(${CISST_USE_FILE})
#   cisst_set_output_path()

#   # Build-tree/export vars, like Optoforce
#   set(sawPSMove_INCLUDE_DIR
#       "${sawPSMove_SOURCE_DIR}/include"
#       "${sawPSMove_BINARY_DIR}/include")
#   set(sawPSMove_HEADER_DIR "${sawPSMove_SOURCE_DIR}/include/sawPSMove")
#   set(sawPSMove_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
#   set(sawPSMove_LIBRARIES sawPSMove)

#   # PS Move env vars
#   set(PSMOVE_INCLUDE_DIR "$ENV{PSMOVE_ROOT}/include")
#   set(PSMOVE_CONFIG_DIR "$ENV{PSMOVE_BUILD}")   # generated headers

#   include_directories(
#     ${sawPSMove_INCLUDE_DIR}
#     ${PSMOVE_INCLUDE_DIR}
#     ${PSMOVE_CONFIG_DIR}
#   )

#   link_directories(${PSMOVE_LIBRARY_DIR})

#   add_library(sawPSMove
#     code/mtsPSMove.cpp
#     ${sawPSMove_HEADER_DIR}/mtsPSMove.h
#     ${sawPSMove_HEADER_DIR}/sawPSMoveExport.h
#     ${sawPSMove_BINARY_DIR}/include/sawPSMove/sawPSMoveRevision.h)

#   set_target_properties(sawPSMove PROPERTIES
#     VERSION ${sawPSMove_VERSION}
#     FOLDER "sawPSMove")

#   target_link_libraries(sawPSMove
#     PUBLIC ${PSMOVE_LIB} ${REQUIRED_CISST_LIBRARIES})

#   # Optional Qt widget (mirrors Optoforce)
#   if(CISST_HAS_QT)
#     add_subdirectory(code/Qt)
#     list(APPEND sawPSMove_LIBRARIES sawPSMoveQt)
#   endif()

#   # Generate the build-tree + installable package config (critical!)
#   cisst_add_config_files(sawPSMove)

#   # Install headers + libs (like Optoforce)
#   install(DIRECTORY
#             "${sawPSMove_SOURCE_DIR}/include/sawPSMove"
#             "${sawPSMove_BINARY_DIR}/include/sawPSMove"
#           DESTINATION include
#           COMPONENT sawPSMove-dev)

#   install(TARGETS sawPSMove COMPONENT sawPSMove
#           RUNTIME DESTINATION bin
#           LIBRARY DESTINATION lib
#           ARCHIVE DESTINATION lib)
# else()
#   message("Info: skipping sawPSMove, cisst not found")
# endif()



cmake_minimum_required (VERSION 3.10)
project (sawPSMove VERSION 0.1.0)

# Core libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstNumerical
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
  set (sawPSMove_LIBRARIES sawPSMove)
  #   # PS Move env vars
  set(PSMOVE_INCLUDE_DIR "$ENV{PSMOVE_ROOT}/include")
  set(PSMOVE_CONFIG_DIR "$ENV{PSMOVE_BUILD}")   # generated headers

  include_directories(
    ${sawPSMove_INCLUDE_DIR}
    ${PSMOVE_INCLUDE_DIR}
    ${PSMOVE_CONFIG_DIR}
  )

#   include_directories (${sawPSMove_INCLUDE_DIR})

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

  cisst_target_link_libraries (sawPSMove ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY "${sawPSMove_SOURCE_DIR}/include/sawPSMove"
                     "${sawPSMove_BINARY_DIR}/include/sawPSMove"
           DESTINATION include
           COMPONENT sawPSMove-dev
           PATTERN .svn EXCLUDE)

  install (TARGETS sawPSMove COMPONENT sawPSMove
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
