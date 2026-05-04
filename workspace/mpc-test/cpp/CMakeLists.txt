# cmake_minimum_required(VERSION 3.16)
# project(my_casadi_app CXX)
# set(CMAKE_CXX_STANDARD 17)

# set(CASADI_ROOT "${CMAKE_SOURCE_DIR}/casadi")
# include_directories("${CASADI_ROOT}/include" "${CASADI_ROOT}/build/include")

# find_library(CASADI_LIB casadi HINTS "${CASADI_ROOT}/build/lib")
# add_executable(myapp src/main.cpp)
# target_link_libraries(myapp PRIVATE ${CASADI_LIB})
# set_target_properties(myapp PROPERTIES BUILD_RPATH "${CASADI_ROOT}/build/lib")

cmake_minimum_required(VERSION 3.16)
project(mpc_casadi CXX)
set(CMAKE_CXX_STANDARD 17)

find_library(CASADI_LIB casadi HINTS "${CMAKE_SOURCE_DIR}/casadi/build/lib")
include_directories("${CMAKE_SOURCE_DIR}/casadi/include" "${CMAKE_SOURCE_DIR}/casadi/build/include")

add_executable(mpc src/main.cpp src/dynamics.cpp)
target_link_libraries(mpc PRIVATE ${CASADI_LIB})
set_target_properties(mpc PROPERTIES BUILD_RPATH "${CMAKE_SOURCE_DIR}/casadi/build/lib")
