cmake_minimum_required(VERSION 3.16)
project(oak_stream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O3")

find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

# building library
add_library(depthai_sensor
    src/DepthAISensor.cpp
)
target_include_directories(depthai_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(depthai_sensor PUBLIC
    depthai::opencv
    ${OpenCV_LIBS}
    Threads::Threads
    Eigen3::Eigen
    Sophus::Sophus
    ${Pangolin_LIBRARIES}
)

# building exe
add_executable(oak_stream 
    src/main.cpp
    src/ImuAttitudeFilter.cpp
)

target_link_libraries(oak_stream PRIVATE
    depthai_sensor
)


# unit test
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
        src/ImuAttitudeFilter.cpp
    )
    target_include_directories(test_eskf PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_eskf PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
    )

    add_test(NAME ESKFTest COMMAND test_eskf)
endif()
