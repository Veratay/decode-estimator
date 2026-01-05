# Tests CMakeLists.txt

# Find GTest
find_package(GTest QUIET)

if(GTest_FOUND)
    message(STATUS "Found GTest, building tests")

    # Test bearing factor
    add_executable(test_bearing_factor test_bearing_factor.cpp)
    target_link_libraries(test_bearing_factor
        PRIVATE
            decode_estimator
            GTest::gtest
            GTest::gtest_main
    )
    add_test(NAME test_bearing_factor COMMAND test_bearing_factor)

    # Test pose estimator
    add_executable(test_pose_estimator test_pose_estimator.cpp)
    target_link_libraries(test_pose_estimator
        PRIVATE
            decode_estimator
            GTest::gtest
            GTest::gtest_main
    )
    add_test(NAME test_pose_estimator COMMAND test_pose_estimator)

else()
    message(STATUS "GTest not found, skipping tests")
endif()
