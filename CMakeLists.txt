cmake_minimum_required(VERSION 3.16)
project(decode_estimator VERSION 1.0.0 LANGUAGES CXX)

# Prefer local CMake modules to avoid broken system GTSAM config packages.
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# C++17 required for Rerun SDK
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(DECODE_ENABLE_RERUN "Enable Rerun visualization" OFF)
option(DECODE_BUILD_EXAMPLES "Build example programs" ON)
option(DECODE_BUILD_TESTS "Build unit tests" ON)

# Release mode for performance (GTSAM is up to 10x faster in Release)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Rerun visualization: ${DECODE_ENABLE_RERUN}")

# Find GTSAM
find_package(GTSAM REQUIRED)

message(STATUS "Found GTSAM: ${GTSAM_INCLUDE_DIR}")

# Optional: Find Rerun SDK
if(DECODE_ENABLE_RERUN)
    include(FetchContent)
    FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/download/0.21.0/rerun_cpp_sdk.zip
    )
    FetchContent_MakeAvailable(rerun_sdk)
    message(STATUS "Rerun SDK enabled")
endif()

# Create library
add_library(${PROJECT_NAME}
    src/pose_estimator.cpp
    src/bearing_factor.cpp
    src/landmark_map.cpp
    src/visualization.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link GTSAM
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        gtsam
)

# Conditional Rerun support
if(DECODE_ENABLE_RERUN)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            rerun_sdk
    )
    target_compile_definitions(${PROJECT_NAME}
        PUBLIC
            DECODE_ENABLE_RERUN=1
    )
else()
    target_compile_definitions(${PROJECT_NAME}
        PUBLIC
            DECODE_ENABLE_RERUN=0
    )
endif()

# Performance flags for Release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -O3
        -DNDEBUG
    )
    # Only use -march=native on x86_64, skip on ARM for cross-compilation compatibility
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        target_compile_options(${PROJECT_NAME} PRIVATE -march=native)
    endif()
endif()

# Examples
if(DECODE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
if(DECODE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation rules
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE decode::
    DESTINATION lib/cmake/${PROJECT_NAME}
)
