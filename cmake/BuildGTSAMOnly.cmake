# BuildGTSAMOnly.cmake - Build GTSAM from source (uses bundled Eigen)
include(FetchContent)

# Force -O2 optimization for ABI compatibility (avoid -O3 which causes Eigen issues)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "Release flags" FORCE)

# Set GTSAM build options - use bundled Eigen for ABI compatibility
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "Don't build examples")
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "Don't build tests")
set(GTSAM_BUILD_DOC OFF CACHE BOOL "Don't build documentation")
set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "Don't build unstable")
set(GTSAM_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "Don't install MATLAB toolbox")
set(GTSAM_USE_SYSTEM_EIGEN OFF CACHE BOOL "Use bundled Eigen for ABI compatibility")
set(GTSAM_WITH_EIGEN_MKL OFF CACHE BOOL "Disable MKL")
set(GTSAM_WITH_TBB ON CACHE BOOL "Enable TBB for consistent allocator")
set(GTSAM_USE_TBB ON CACHE BOOL "Use TBB allocator consistently")
set(GTSAM_BUILD_STATIC_LIBRARY ON CACHE BOOL "Build GTSAM as static library")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libraries for CM4 deployment")

# Fetch GTSAM from source
FetchContent_Declare(
    gtsam
    URL https://github.com/borglab/gtsam/archive/refs/tags/4.2.0.tar.gz
)

# Make GTSAM available
FetchContent_MakeAvailable(gtsam)

# Set variables for parent project - use GTSAM's bundled Eigen
set(GTSAM_INCLUDE_DIR ${gtsam_SOURCE_DIR})
set(GTSAM_EIGEN_INCLUDE_DIR ${gtsam_SOURCE_DIR}/gtsam/3rdparty/Eigen)
set(GTSAM_LIBRARIES gtsam)

# Find TBB to ensure consistent allocator usage
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc REQUIRED)

# Create a custom target to ensure GTSAM is built
add_custom_target(gtsam_build ALL)
add_dependencies(gtsam_build gtsam)

# Set TBB variables for parent project
set(TBB_INCLUDE_DIRS ${TBB_INCLUDE_DIRS})
set(TBB_LIBRARIES ${TBB_LIBRARIES})