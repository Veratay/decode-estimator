cmake_minimum_required(VERSION 3.16)

project(decode_estimator LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(DECODE_ENABLE_RERUN "Enable Rerun visualization" OFF)
option(DECODE_BUILD_TESTS "Build unit tests (host only)" OFF)
option(DECODE_BUILD_EXAMPLES "Build examples" ON)
option(DECODE_FORCE_STATIC "Prefer static libraries" ON)
option(DECODE_FETCH_GTEST "Fetch GoogleTest locally when tests enabled" ON)
option(DECODE_BUILD_FOR_ANDROID "Build for Android using NDK" OFF)

set(DECODE_EIGEN_GIT_TAG "3.4.0" CACHE STRING "Eigen git tag")
set(DECODE_GTSAM_GIT_TAG "4.3a1" CACHE STRING "GTSAM git tag")
set(DECODE_GTEST_GIT_TAG "v1.14.0" CACHE STRING "GoogleTest git tag")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Collect Android toolchain arguments for passing to ExternalProject_Add
set(EXTERNAL_ANDROID_ARGS "")
foreach(var ANDROID_ABI ANDROID_PLATFORM ANDROID_STL ANDROID_ARM_NEON ANDROID_PIE)
    if(DEFINED ${var})
        list(APPEND EXTERNAL_ANDROID_ARGS "-D${var}=${${var}}")
    endif()
endforeach()

# Find JNI for Android builds or if available on host
find_package(JNI QUIET)

if(DECODE_FORCE_STATIC)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    if(UNIX AND NOT APPLE)
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ${CMAKE_FIND_LIBRARY_SUFFIXES})
    endif()
endif()

list(PREPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG OFF)

include(ExternalProject)
include(FetchContent)

set(DECODE_DEPS_DIR "${CMAKE_BINARY_DIR}/_deps")
set(DECODE_EIGEN_INSTALL_DIR "${DECODE_DEPS_DIR}/eigen-install")
set(DECODE_GTSAM_INSTALL_DIR "${DECODE_DEPS_DIR}/gtsam-install")
file(MAKE_DIRECTORY
    "${DECODE_EIGEN_INSTALL_DIR}/include/eigen3"
    "${DECODE_GTSAM_INSTALL_DIR}/include"
    "${DECODE_GTSAM_INSTALL_DIR}/lib"
)

ExternalProject_Add(
    eigen_ep
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG ${DECODE_EIGEN_GIT_TAG}
    UPDATE_DISCONNECTED 1
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${DECODE_EIGEN_INSTALL_DIR}
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DBUILD_TESTING=OFF
        -DEIGEN_BUILD_DOC=OFF
        -DEIGEN_BUILD_PKGCONFIG=OFF
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        ${EXTERNAL_ANDROID_ARGS}
)

add_library(Eigen3::Eigen INTERFACE IMPORTED)
set(EIGEN3_INCLUDE_DIR "${DECODE_EIGEN_INSTALL_DIR}/include/eigen3")
set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
)
add_dependencies(Eigen3::Eigen eigen_ep)

set(_gtsam_cmake_args
    -DCMAKE_INSTALL_PREFIX=${DECODE_GTSAM_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DGTSAM_USE_SYSTEM_EIGEN=ON
    -DEigen3_DIR=${DECODE_EIGEN_INSTALL_DIR}/share/eigen3/cmake
    -DGTSAM_BUILD_SHARED_LIBS=OFF
    -DBUILD_SHARED_LIBS=OFF
    -DGTSAM_BUILD_TESTS=OFF
    -DGTSAM_BUILD_UNSTABLE=OFF
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
    -DGTSAM_BUILD_TYPE_POSTFIXES=OFF
    -DGTSAM_USE_TBB=OFF
    -DGTSAM_WITH_TBB=OFF
    -DGTSAM_WITH_EIGEN_MKL=OFF
    -DGTSAM_WITH_EIGEN_MKL_OPENMP=OFF
    -DGTSAM_BUILD_PYTHON=OFF
    -DGTSAM_USE_BOOST_FEATURES=OFF
    -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)

if(CMAKE_SYSTEM_NAME STREQUAL "Android" OR DECODE_BUILD_FOR_ANDROID)
    list(APPEND _gtsam_cmake_args -DHAVE_EXECINFO_H=0)
endif()

if(CMAKE_TOOLCHAIN_FILE)
    list(APPEND _gtsam_cmake_args -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
endif()
list(APPEND _gtsam_cmake_args ${EXTERNAL_ANDROID_ARGS})

# Determine GTSAM dependencies
set(_gtsam_depends eigen_ep)

ExternalProject_Add(
    gtsam_ep
    GIT_REPOSITORY https://github.com/borglab/gtsam.git
    GIT_TAG ${DECODE_GTSAM_GIT_TAG}
    UPDATE_DISCONNECTED 1
    DEPENDS ${_gtsam_depends}
    CMAKE_ARGS ${_gtsam_cmake_args}
)

add_library(gtsam STATIC IMPORTED GLOBAL)
set_target_properties(gtsam PROPERTIES
    IMPORTED_LOCATION "${DECODE_GTSAM_INSTALL_DIR}/lib/libgtsam.a"
    INTERFACE_INCLUDE_DIRECTORIES "${DECODE_GTSAM_INSTALL_DIR}/include"
)
add_dependencies(gtsam gtsam_ep)

add_library(metis-gtsam STATIC IMPORTED GLOBAL)
set_target_properties(metis-gtsam PROPERTIES
    IMPORTED_LOCATION "${DECODE_GTSAM_INSTALL_DIR}/lib/libmetis-gtsam.a"
)
add_dependencies(metis-gtsam gtsam_ep)

find_package(Threads REQUIRED)

if(DECODE_ENABLE_RERUN)
    FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(rerun_sdk)
endif()

add_library(decode_estimator STATIC
    src/ekf_localizer.cpp
    src/pose_estimator.cpp
    src/bearing_factor.cpp
    src/range_factor.cpp
    src/landmark_map.cpp
    src/visualization.cpp
)

target_include_directories(decode_estimator
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(decode_estimator PUBLIC cxx_std_17)
target_compile_definitions(decode_estimator
    PUBLIC
        DECODE_ENABLE_RERUN=$<BOOL:${DECODE_ENABLE_RERUN}>
        DECODE_BUILD_FOR_ANDROID=$<BOOL:${DECODE_BUILD_FOR_ANDROID}>
)

target_link_libraries(decode_estimator
    PUBLIC
        gtsam
        metis-gtsam
        Eigen3::Eigen
        Threads::Threads
)

if(DECODE_ENABLE_RERUN)
    if(TARGET rerun_sdk)
        target_link_libraries(decode_estimator PUBLIC rerun_sdk)
    elseif(TARGET rerun::rerun_sdk)
        target_link_libraries(decode_estimator PUBLIC rerun::rerun_sdk)
    endif()
endif()

# Build JNI library for Android or if JNI is available on host
if(JNI_FOUND OR CMAKE_SYSTEM_NAME STREQUAL "Android" OR DECODE_BUILD_FOR_ANDROID)
    add_library(decode_estimator_jni SHARED
        src/ekf_localizer.cpp
        src/pose_estimator.cpp
        src/bearing_factor.cpp
        src/range_factor.cpp
        src/landmark_map.cpp
        src/visualization.cpp
        src/pose_estimator_jni.cpp
    )

    target_include_directories(decode_estimator_jni
        PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    if(JNI_FOUND)
        target_include_directories(decode_estimator_jni PRIVATE ${JNI_INCLUDE_DIRS})
        target_link_libraries(decode_estimator_jni PRIVATE ${JNI_LIBRARIES})
    endif()

    target_compile_features(decode_estimator_jni PUBLIC cxx_std_17)
    target_compile_definitions(decode_estimator_jni
        PUBLIC
            DECODE_ENABLE_RERUN=$<BOOL:${DECODE_ENABLE_RERUN}>
            DECODE_BUILD_FOR_ANDROID=$<BOOL:${DECODE_BUILD_FOR_ANDROID}>
    )

    target_link_libraries(decode_estimator_jni
        PUBLIC
            gtsam
            metis-gtsam
            Eigen3::Eigen
            Threads::Threads
    )

    if(NOT WIN32)
        target_link_libraries(decode_estimator_jni PRIVATE m dl)
    endif()

    # Add gc-sections for smaller binary size
    target_link_options(decode_estimator_jni PRIVATE -Wl,--gc-sections)
    target_compile_options(decode_estimator_jni PRIVATE -ffunction-sections -fdata-sections)
endif()

if(DECODE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(DECODE_BUILD_TESTS)
    if(CMAKE_CROSSCOMPILING OR DECODE_BUILD_FOR_ANDROID)
        message(FATAL_ERROR "DECODE_BUILD_TESTS=ON is not supported when cross-compiling or building for Android.")
    endif()

    include(CTest)
    enable_testing()

    if(DECODE_FETCH_GTEST)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG ${DECODE_GTEST_GIT_TAG}
        )
        FetchContent_MakeAvailable(googletest)
    else()
        find_package(GTest REQUIRED)
    endif()

    add_subdirectory(tests)
endif()
