set(REQUIRED_CISST_LIBRARIES
    cisstCommon
    cisstVector
    cisstOSAbstraction
    cisstMultiTask
    cisstParameterTypes
)
find_package(cisst 1.0.11 COMPONENTS ${REQUIRED_CISST_LIBRARIES} REQUIRED)
include(${CISST_USE_FILE})

# We built sawPSMove in a parent dir via add_subdirectory()
add_executable(PSMoveExample main.cpp)
cisst_target_link_libraries(PSMoveExample ${REQUIRED_CISST_LIBRARIES})
target_link_libraries(PSMoveExample sawPSMove)
