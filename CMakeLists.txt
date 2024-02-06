cmake_minimum_required(VERSION 3.1.0)
project(motion_profile_generators VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type)
endif()

set (CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)

find_package(GTest REQUIRED)
find_package(Matplot++ REQUIRED)

add_library(
    ${CMAKE_PROJECT_NAME}
    SHARED
    src/triangular_profile.cpp
    src/motion_profile_controller.cpp
)
target_compile_features(${CMAKE_PROJECT_NAME} PUBLIC cxx_std_17)
#target_compile_options(
#    ${CMAKE_PROJECT_NAME}
#    PUBLIC
#    -Wall
#)
target_include_directories(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>

)
install(
    DIRECTORY include/
    DESTINATION /usr/local/include/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION /usr/local/bin/${PROJECT_NAME}
    ARCHIVE DESTINATION /usr/local/lib/${PROJECT_NAME}
    LIBRARY DESTINATION /usr/local/lib/${PROJECT_NAME}
)

#add_subdirectory(test ${CMAKE_CURRENT_SOURCE_DIR}/build/test)

add_executable(triangular_profile_test test/triangular_profile_test/main.cpp)
target_link_libraries(triangular_profile_test ${CMAKE_PROJECT_NAME} ${GTEST_LIBRARIES}  Matplot++::matplot)
target_include_directories(triangular_profile_test PUBLIC ${GTEST_INCLUDE_DIRS} include/)

add_executable(trapezoidal_profile_test test/trapezoidal_profile_test/main.cpp)
target_link_libraries(trapezoidal_profile_test ${CMAKE_PROJECT_NAME} ${GTEST_LIBRARIES}  Matplot++::matplot)
target_include_directories(trapezoidal_profile_test PUBLIC ${GTEST_INCLUDE_DIRS} include/)

add_executable(scurve_profile_test test/s_curve_profile_test/main.cpp)
target_link_libraries(scurve_profile_test ${CMAKE_PROJECT_NAME} ${GTEST_LIBRARIES}  Matplot++::matplot)
target_include_directories(scurve_profile_test PUBLIC ${GTEST_INCLUDE_DIRS} include/)