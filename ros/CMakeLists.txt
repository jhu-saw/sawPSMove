#
# (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---
cmake_minimum_required(VERSION 3.10)
project(psmove_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# cisst
set(REQUIRED_CISST_LIBRARIES
    cisstCommon
    cisstCommonQt
    cisstVector
    cisstVectorQt
    cisstOSAbstraction
    cisstMultiTask
    cisstMultiTaskQt
    cisstParameterTypes
    cisstQt
)
find_package(cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})
include(${CISST_USE_FILE})

# sawPSMove was built in components/
# so we can just link the target
add_executable(psmove_node src/psmove_node.cpp)
target_link_libraries(psmove_node
  sawPSMove
  ${REQUIRED_CISST_LIBRARIES}
)

# Qt optional
find_package(Qt5 QUIET COMPONENTS Widgets)
if (Qt5Widgets_FOUND)
  target_link_libraries(psmove_node sawPSMoveQt Qt5::Widgets)
endif()

install(TARGETS psmove_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
