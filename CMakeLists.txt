cmake_minimum_required(VERSION 3.16)
project(ahrs_eskf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O3")

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(Pangolin REQUIRED)

#build with csv_9dof
add_executable(csv_9dof
    src/app_csv_9dof.cpp
    src/core/ImuAttitudeFilter.cpp
)
target_link_libraries(csv_9dof PRIVATE
    Threads::Threads
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)
target_include_directories(csv_9dof PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

#build with depthai_6dof
option(BUILD_DEPTHAI_APP "Build DepthAI 6DoF app" OFF)
if(BUILD_DEPTHAI_APP)
    find_package(depthai CONFIG REQUIRED)

    add_executable(depthai_6dof
        src/app_depthai_6dof.cpp
        src/sensor/DepthAISensor.cpp
        src/core/ImuAttitudeFilter.cpp
    )
    target_include_directories(depthai_6dof PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
        ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
    )
    target_link_libraries(depthai_6dof PRIVATE
        depthai::opencv
        Threads::Threads
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
    )
endif()

# build with unit test
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/release-1.12.1.zip
    )
    FetchContent_MakeAvailable(googletest)

    add_executable(test_eskf
        tests/test_eskf.cpp
        src/core/ImuAttitudeFilter.cpp
    )
    target_include_directories(test_eskf PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    )
    target_link_libraries(test_eskf PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
    )

    add_test(NAME ESKFTest COMMAND test_eskf)
endif()
