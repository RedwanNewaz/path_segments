include(cmake/CPM.cmake)
CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
CPMAddPackage("gh:fmtlib/fmt#7.1.3")


include_directories(${CMAKE_CURRENT_BINARY_DIR}/../_deps/yaml-cpp-src/include)

add_library(input_generator input_generator.cpp)
target_compile_features(input_generator PRIVATE cxx_std_17)
target_link_libraries(input_generator yaml-cpp fmt)

add_executable(input_gen main.cpp input_generator.cpp)
target_compile_features(input_gen PRIVATE cxx_std_17)
target_link_libraries(input_gen yaml-cpp fmt)