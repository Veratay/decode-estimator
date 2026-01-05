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
option(DECODE_USE_LOCAL_DEPS "Build Eigen and GTSAM from source" OFF)

# Set parallel build level to nproc if not specified
if(NOT CMAKE_BUILD_PARALLEL_LEVEL)
    execute_process(COMMAND nproc OUTPUT_VARIABLE NPROC OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CMAKE_BUILD_PARALLEL_LEVEL ${NPROC} CACHE STRING "Number of parallel build jobs")
endif()

# Release mode for performance (GTSAM is up to 10x faster in Release)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Rerun visualization: ${DECODE_ENABLE_RERUN}")
message(STATUS "Use local dependencies: ${DECODE_USE_LOCAL_DEPS}")

# Build dependencies from source or find system packages
if(DECODE_USE_LOCAL_DEPS)
    message(STATUS "Building GTSAM and Eigen from source...")
    include(BuildGTSAMOnly)
    message(STATUS "Local Eigen: ${EIGEN3_INCLUDE_DIR}")
    message(STATUS "Local GTSAM: ${GTSAM_INCLUDE_DIR}")
else()
    # Find system Eigen3 (required for GTSAM)
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
    # Find system GTSAM
    find_package(GTSAM REQUIRED)
    message(STATUS "Found GTSAM: ${GTSAM_INCLUDE_DIR}")
    # Find TBB to ensure consistent allocator usage
    find_package(TBB 4.4 COMPONENTS tbb tbbmalloc REQUIRED)
    message(STATUS "Found TBB: ${TBB_INCLUDE_DIRS}")
endif()

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

# Create library (STATIC for CM4 deployment)
add_library(${PROJECT_NAME} STATIC
    src/pose_estimator.cpp
    src/bearing_factor.cpp
    src/landmark_map.cpp
    src/visualization.cpp
)

# Add dependencies on external projects if building locally
if(DECODE_USE_LOCAL_DEPS)
    add_dependencies(${PROJECT_NAME} gtsam_build)
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            ${TBB_LIBRARIES}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC ${TBB_INCLUDE_DIRS})
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Add Eigen include directory (use BUILD_INTERFACE for local deps to avoid install issues)
if(DECODE_USE_LOCAL_DEPS)
    # Use GTSAM's bundled Eigen for ABI compatibility
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${GTSAM_EIGEN_INCLUDE_DIR}>)
else()
    target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# Link GTSAM and TBB for consistent allocator usage
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        gtsam
        tbb
        tbbmalloc
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
# Use -O2 instead of -O3 to avoid Eigen ABI issues with aggressive optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -O2
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
