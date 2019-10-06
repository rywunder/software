# find required packages
find_package(OpenCV REQUIRED)
find_package(Apriltags REQUIRED)
find_package(Eigen3)

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(SYSTEM
    ${APRILTAGS_INCLUDE_DIRS}
)

add_library(AprilTagWrapper SHARED AprilTagWrapper.cpp)

add_executable(AprilTagTest AprilTagWrapperMain.cpp)

target_link_libraries(AprilTagWrapper
    ${OpenCV_LIBS}
    ${APRILTAGS_LIBRARIES}

)
add_executable(AprilTagExample opencv_demo.cc)

target_link_libraries(AprilTagExample
    ${OpenCV_LIBS}
    ${APRILTAGS_LIBRARIES}
)

target_link_libraries(AprilTagTest
    ${OpenCV_LIBS}
    ${APRILTAGS_LIBRARIES}
    AprilTagWrapper
)
