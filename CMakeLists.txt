cmake_minimum_required(VERSION 3.10)
project(sawPSMove VERSION 0.1.0)

# ---- cisst ----
find_package(cisst REQUIRED)
include(${CISST_USE_FILE})

# Packaging (same pattern as Optoforce)
cisst_cpack_settings(
  VENDOR "YourOrg"
  MAINTAINER "you@example.com")

# ---- Respect your environment vars for PS Move API ----
if(NOT DEFINED ENV{PSMOVE_ROOT})
  message(FATAL_ERROR "Please set PSMOVE_ROOT to your psmoveapi root")
endif()
if(NOT DEFINED ENV{PSMOVE_BUILD})
  message(FATAL_ERROR "Please set PSMOVE_BUILD to your psmoveapi/build")
endif()
set(PSMOVE_INCLUDE_DIR "$ENV{PSMOVE_ROOT}/include")
set(PSMOVE_LIBRARY_DIR "$ENV{PSMOVE_BUILD}")

# Try both lib names
find_library(PSMOVE_LIB NAMES psmoveapi psmove HINTS ${PSMOVE_LIBRARY_DIR})
if(NOT PSMOVE_LIB)
  message(FATAL_ERROR "Could not find PS Move API library in ${PSMOVE_LIBRARY_DIR}")
endif()

# Build components first (this generates the package config)
add_subdirectory(components)

# Make ROS/examples able to find the build-tree package immediately,
# exactly like sawOptoforce sets sawOptoforceSensor_DIR to its components build dir.
set(sawPSMove_DIR "${CMAKE_BINARY_DIR}/components")

# Then build examples and ROS
# add_subdirectory(examples)
add_subdirectory(ros)

include(CPack)
cpack_add_component(sawPSMove)
cpack_add_component(sawPSMove-Examples DEPENDS sawPSMove)
cpack_add_component(sawPSMove-ROS DEPENDS sawPSMove)
