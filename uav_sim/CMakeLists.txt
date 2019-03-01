cmake_minimum_required(VERSION 3.4)
project(uav_sim)

add_subdirectory(lib/dynamics)

find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib uav_msgs)
find_package(Eigen3 REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS
    osgDB
    osgGA
    osgUtil
    osgViewer
    osgText
    )

catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS roscpp std_msgs roslib uav_msgs
    )

get_target_property(QtCore_location Qt5::Core LOCATION)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
    lib/dynamics/include
    lib/dynamics/include/geometry/include/
    )

file(COPY obj DESTINATION devel/lib/uav_sim)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE MAVSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS src/*.cpp)
file(GLOB MAVFORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB MAVICONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} icons/*.qrc)
file(GLOB_RECURSE MAVMOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS include/uav_sim/*.hpp)
file(GLOB_RECURSE TESTSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    FOLLOW_SYMLINKS test/*.cpp)

QT5_WRAP_UI(QT_FORMS_HPP ${MAVFORMS})

add_executable(uav_sim
   ${MAVSOURCE}
   ${MAVICONS}
   ${MAVOBJ}
   ${MAVMOC}
   ${QT_FORMS_HPP}
   )

target_link_libraries(uav_sim
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    Qt5::Widgets
    Qt5::Gui
    Qt5::Core
    Dynamics
    )

install(TARGETS uav_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#file(GLOB_RECURSE APSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/autopilot/*.cpp)
#file(GLOB_RECURSE APMOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/autopilot/*.hpp)
#
#add_executable(autopilot
#    ${APSOURCE}
#    ${APMOC}
#    )
#target_link_libraries(autopilot
#    ${catkin_LIBRARIES}
#    ${EIGEN3_LIBRARIES}
#    Dynamics
#    )
