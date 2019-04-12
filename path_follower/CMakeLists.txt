cmake_minimum_required(VERSION 3.4)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs uav_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS roscpp std_msgs uav_msgs
    )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
    )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB_RECURSE PFSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE PFMOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS include/path_follower/*.hpp)

add_executable(path_follower
    ${PFSOURCE}
    ${PFMOC}
    )
target_link_libraries(path_follower
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    )
