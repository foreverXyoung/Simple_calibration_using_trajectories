cmake_minimum_required(VERSION 3.10)
project(Simple_calibration_using_trajectories)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)

# Optional: GoogleTest for unit tests
find_package(GTest QUIET)
option(BUILD_TESTS "Build unit tests" ${GTest_FOUND})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDEDIR}
    ${CERES_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

# Main executable
add_executable(${PROJECT_NAME}_lidar_rtk
    app/calib_lidar_rtk.cpp
    src/hand_eye_calibrator.cpp
)

target_link_libraries(${PROJECT_NAME}_lidar_rtk
    Eigen3::Eigen
    yaml-cpp
    ${CERES_LIBRARIES}
    ${GLOG_LIBRARIES}
    Threads::Threads
)

set_target_properties(${PROJECT_NAME}_lidar_rtk PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
)

# Unit tests (optional)
if(BUILD_TESTS AND GTest_FOUND)
    enable_testing()
    include_directories(${GTEST_INCLUDE_DIRS})

    add_executable(run_tests
        tests/test_calibrator.cpp
        src/hand_eye_calibrator.cpp
    )

    target_link_libraries(run_tests
        Eigen3::Eigen
        yaml-cpp
        ${CERES_LIBRARIES}
        ${GLOG_LIBRARIES}
        Threads::Threads
        ${GTEST_LIBRARIES}
    )

    set_target_properties(run_tests PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
    )

    add_test(NAME CalibrationTests COMMAND run_tests)
endif()
