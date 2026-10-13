cmake_minimum_required(VERSION 3.22)
project(robot_math VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(robot_math src/se2.cpp)
target_link_libraries(robot_math PUBLIC Eigen3::Eigen)
target_include_directories(robot_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(robot_math PUBLIC cxx_std_17)
target_compile_options(robot_math PRIVATE -Wall -Wextra)

enable_testing()
find_package(GTest REQUIRED)
add_executable(robot_math_tests tests/test_se2.cpp)
target_link_libraries(robot_math_tests PRIVATE robot_math GTest::gtest_main)
target_compile_options(robot_math_tests PRIVATE -Wall -Wextra)
include(GoogleTest)
gtest_discover_tests(robot_math_tests)