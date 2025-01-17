if(NOT BUILD_GCS)
    return()
endif()

include(FindPkgConfig REQUIRED)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

find_package(ZCM REQUIRED)

include_directories(
    ${GTKMM_INCLUDE_DIRS}
    ${ZCM_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
)

add_executable(maav-gcs
    maav-gcs.cpp
    GCS.cpp
    GCSConsts.cpp
    StatusFrame.cpp
    CommandFrame.cpp
    TuningFrame.cpp
    GlibZCM.cpp
    XboxController.cpp
)

target_link_libraries(maav-gcs
    maav-gamepad
	maav-message
	maav-common
    ${GTKMM_LIBRARIES}
    ${ZCM_LIBRARY}
    ${YAMLCPP_LIBRARY}
)

if(BUILD_TESTS)
    add_executable(fake-ctrl
        fake-ctrl.cpp
    )

    target_link_libraries(fake-ctrl
        ${ZCM_LIBRARY}
        ${SOFTWARE_LIBS}
    )
endif()
