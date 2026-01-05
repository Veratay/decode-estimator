# Examples CMakeLists.txt

add_executable(simple_localization simple_localization.cpp)
target_link_libraries(simple_localization PRIVATE decode_estimator)
