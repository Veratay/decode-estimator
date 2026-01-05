# BuildGTSAMOnly.cmake - Build GTSAM from source
include(FetchContent)

# Set GTSAM build options
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "Don't build examples")
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "Don't build tests")
set(GTSAM_BUILD_DOC OFF CACHE BOOL "Don't build documentation")
set(GTSAM_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "Don't install MATLAB toolbox")
set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "Use system Eigen")
set(GTSAM_USE_TBB OFF CACHE BOOL "Disable TBB to avoid allocator conflicts")
set(GTSAM_USE_POSIX_THREADS ON CACHE BOOL "Use POSIX threads instead")
set(GTSAM_DISABLE_TBB_MALLOC ON CACHE BOOL "Disable TBB malloc proxy")
set(TBB_ENABLE_MALLOC_PROXY OFF CACHE BOOL "Disable TBB malloc proxy globally")
set(GTSAM_FORCE_NO_TBB ON CACHE BOOL "Force disable TBB completely")

# Fetch GTSAM from source
FetchContent_Declare(
    gtsam
    URL https://github.com/borglab/gtsam/archive/refs/tags/4.2.0.tar.gz
)

# Make GTSAM available
FetchContent_MakeAvailable(gtsam)

# Set variables for parent project
set(GTSAM_INCLUDE_DIR ${gtsam_SOURCE_DIR}/include)
set(GTSAM_LIBRARIES gtsam)

# Create a custom target to ensure GTSAM is built
add_custom_target(gtsam_build ALL)
add_dependencies(gtsam_build gtsam)