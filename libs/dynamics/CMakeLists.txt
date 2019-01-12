cmake_minimum_required(VERSION 3.4)
project(DynamicsLib)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    include
    include/3rd_party
    )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

IF (NOT TARGET gtest)
    # Prevent GoogleTest from overriding our compiler/linker options
    # when building with Visual Studio
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory(../googletest)
ENDIF()

add_library(Dynamics STATIC
    include/3rd_party/quat.hpp
    include/3rd_party/solver.h
    include/3rd_party/solver.c
    include/3rd_party/util.c
    include/3rd_party/matrix_support.c
    include/3rd_party/ldl.c
    include/types.hpp
    include/drone.hpp
    include/controller.hpp
    src/drone.cpp
    src/controller.cpp
    )

set(TEST_NAME ${PROJECT_NAME}_UnitTest)

add_executable(${TEST_NAME}
    src/dynamics_unittests.cpp
    )

target_link_libraries(${TEST_NAME}
    ${catkin_LIBRARIES}
    Dynamics
    gtest
    gtest_main
    )
