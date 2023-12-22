cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0135 NEW)
project(visibility_task_planning)
set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(FCL REQUIRED)
find_package(Boost COMPONENTS filesystem program_options python3 REQUIRED)


include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${fcl_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_BINARY_DIR}/install/include)
link_directories(${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/install/lib)


add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        ${fcl_LIBRARIES}
        ${Boost_LIBRARIES}
        )

option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    include(cmake/CPM.cmake)
    CPMAddPackage(
            NAME googletest
            GITHUB_REPOSITORY google/googletest
            GIT_TAG release-1.12.1
            VERSION 1.12.1
            OPTIONS "INSTALL_GTEST OFF" "gtest_force_shared_crt"
    )
    add_executable(collision_checker_test test/test_collision_checker.cpp)
    target_link_libraries(collision_checker_test gtest gtest_main gmock Eigen3::Eigen ${fcl_LIBRARIES})
    enable_testing()
    add_test(collision_checker_test collision_checker_test)
endif(BUILD_TESTS)
