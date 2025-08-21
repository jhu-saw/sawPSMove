# sawPSMove/components/code/Qt/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# cisst Qt stacks needed by the widget (same set OptoForce uses)
set(REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstMultiTaskQt
  cisstQt)

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include(${CISST_USE_FILE})

# Qt only in this sublibrary
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

# Header dir for MOC + exports
set(sawPSMove_HEADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../include/sawPSMove")

# Build the Qt sublibrary (separate target, avoids CMP0079 issues)
add_library(sawPSMoveQt
  ${sawPSMove_HEADER_DIR}/sawPSMoveQtExport.h
  ${sawPSMove_HEADER_DIR}/mtsPSMoveQtWidget.h
  mtsPSMoveQtWidget.cpp
)

target_include_directories(sawPSMoveQt
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

# >>> Use PLAIN signature to match cisst_target_link_libraries <<<
target_link_libraries(sawPSMoveQt Qt5::Widgets)
target_compile_definitions(sawPSMoveQt PRIVATE -DQT_NO_KEYWORDS)

# cisst adds its libs with the plain signature too
cisst_target_link_libraries(sawPSMoveQt ${REQUIRED_CISST_LIBRARIES})

set_target_properties(sawPSMoveQt PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  POSITION_INDEPENDENT_CODE ON
)

# Install the Qt sublibrary
install(TARGETS sawPSMoveQt
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
