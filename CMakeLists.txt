cmake_minimum_required(VERSION 3.10)
project(path_segments)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)
find_package(Boost COMPONENTS filesystem program_options python3 REQUIRED)
find_package(FCL REQUIRED)
include_directories(include ${fcl_INCLUDE_DIRS})

set(SRC_FILES
    include/collision_checker.h
    include/path_decomposition.h
        include/conflict_based_decomposer.h
        src/cbd_ta/conflict_based_decomposer.cpp
    )
add_executable(path_segments main.cpp ${SRC_FILES})
target_link_libraries(path_segments
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        ${fcl_LIBRARIES}
        )

add_library(cbd_ta MODULE  SHARED src/cbd_ta/main_cbd_py.cpp ${SRC_FILES})
target_link_libraries(cbd_ta
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        ${fcl_LIBRARIES}
        ${PYTHON_LIBRARIES}
        )

target_include_directories(cbd_ta PRIVATE ${PYTHON_INCLUDE_DIRS} include)

# Copy the __init__.py file
configure_file(src/cbd_ta/__init__.py ${CMAKE_CURRENT_BINARY_DIR}/src/__init__.py COPYONLY)

# Suppress prefix "lib" because Python does not allow this prefix
set_target_properties(cbd_ta PROPERTIES PREFIX "")