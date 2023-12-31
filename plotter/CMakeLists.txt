find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
# numpy.get_include()
set(PYTHON_INCLUDE_DIR 
/home/codespace/.local/lib/python3.10/site-packages/numpy/core/include
/home/codespace/.python/current/include/python3.10/)

include_directories(
    ${PYTHON_INCLUDE_DIR}
)
link_directories(/home/codespace/.python/current/lib)

add_executable(mtsp_plotter main.cpp)

target_link_libraries(mtsp_plotter -lpython3.10 yaml-cpp fmt -pthread)