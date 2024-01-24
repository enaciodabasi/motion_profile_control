cmake_minimum_required(VERSION 3.1.0)
project(test)

set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)

set(motion_profile_generators /home/naci/motion_profile_generators/build/libmotion_profile_generators.so)
set(motion_profile_generators_INCLUDE_DIRS /usr/local/include/motion_profile_generators/)

add_executable(triangular_profile_test triangular_profile_test/main.cpp)
target_link_libraries(triangular_profile_test ${motion_profile_generators} ${GTEST_LIBRARIES})
target_include_directories(triangular_profile_test PUBLIC ${GTEST_INCLUDE_DIRS} ${motion_profile_generators_INCLUDE_DIRS})