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
project (sawPSMoveCore VERSION 0.1.0)

find_package (cisst 1.5 REQUIRED)
cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "anton.deguet@jhu.edu")

add_subdirectory (components)

set (sawPSMove_DIR "${sawPSMoveCore_BINARY_DIR}/components")
add_subdirectory (examples)

include (CPack)
cpack_add_component (sawPSMove)
cpack_add_component (sawPSMove-dev
                     DEPENDS sawPSMove)
cpack_add_component (sawPSMove-Qt
                     DEPENDS sawPSMove)
cpack_add_component (sawPSMove-Examples
                     DEPENDS sawPSMove sawPSMove-Qt)
