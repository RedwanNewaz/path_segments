cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0135 NEW)
project(visibility_task_planning)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS "-O3 -Wall")


include_directories(
        include
        LKH-3.0.6/SRC/INCLUDE
)

file( GLOB SRCS LKH-3.0.6/SRC/*.c )

add_library( lkh_mtsp_solver SHARED
        ${SRCS}
        src/lkh3_interface.cpp
)
target_link_libraries(lkh_mtsp_solver -lm)

add_executable(mtsp main.cpp)

target_link_libraries(mtsp lkh_mtsp_solver)
