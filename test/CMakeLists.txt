cmake_minimum_required(VERSION 3.1.0)
project(test)

set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)
find_package(MotionProfileGenerators REQUIRED)

set(motion_profile_generators /usr/local/lib/MotionProfileGenerators/libMotionProfileGenerators.so)
set(motion_profile_generators_INCLUDE_DIRS /usr/local/include/MotionProfileGenerators/)
#
#add_executable(triangular_profile_test triangular_profile_test/main.cpp)
#target_link_libraries(triangular_profile_test ${motion_profile_generators} ${GTEST_LIBRARIES})
#target_include_directories(triangular_profile_test PUBLIC ${GTEST_INCLUDE_DIRS} ${motion_profile_generators_INCLUDE_DIRS})

add_executable(controller_test controller_class_test/main.cpp)
target_link_libraries(controller_test ${motion_profile_generators} ${GTEST_LIBRARIES})
target_include_directories(controller_test PUBLIC ${GTEST_INCLUDE_DIRS} ${motion_profile_generators_INCLUDE_DIRS})

install(
    TARGETS
    controller_test
    DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../build
)