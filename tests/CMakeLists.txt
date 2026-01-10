# Tests CMakeLists.txt

if(NOT TARGET GTest::gtest)
    message(FATAL_ERROR "GTest targets are missing. Enable DECODE_FETCH_GTEST or set GTest_DIR.")
endif()

# Test pose estimator
add_executable(test_pose_estimator test_pose_estimator.cpp)
target_link_libraries(test_pose_estimator
    PRIVATE
        decode_estimator
        GTest::gtest
        GTest::gtest_main
)
add_test(NAME test_pose_estimator COMMAND test_pose_estimator)
