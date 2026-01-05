find_path(
    GTSAM_INCLUDE_DIR
    NAMES gtsam/base/Matrix.h
    HINTS
        ${GTSAM_DIR}
        ${GTSAM_ROOT}
    PATH_SUFFIXES include
)

find_library(
    GTSAM_LIBRARY
    NAMES gtsam
    HINTS
        ${GTSAM_DIR}
        ${GTSAM_ROOT}
    PATH_SUFFIXES lib lib64
)

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    GTSAM
    REQUIRED_VARS GTSAM_INCLUDE_DIR GTSAM_LIBRARY
)

if(GTSAM_FOUND)
    set(GTSAM_INCLUDE_DIRS "${GTSAM_INCLUDE_DIR}")
    set(GTSAM_LIBRARIES "${GTSAM_LIBRARY}")

    if(NOT TARGET gtsam)
        add_library(gtsam UNKNOWN IMPORTED)
        set_target_properties(gtsam PROPERTIES
            IMPORTED_LOCATION "${GTSAM_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GTSAM_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "Eigen3::Eigen;TBB::tbb"
        )
    endif()
endif()
