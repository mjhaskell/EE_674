cmake_minimum_required(VERSION 3.4)
project(fixedwing_sim)

find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5Core)
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS
    osgDB
    osgGA
    osgUtil
    osgViewer
    osgText
    )

get_target_property(QtCore_location Qt5::Core LOCATION)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
#    /usr/local/include/eigen3
    include
    )

#catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp std_msgs)
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp std_msgs roslib)

file(COPY obj DESTINATION devel/lib/fixedwing_sim)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE MYSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB MYFORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB MYICONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} icons/*.qrc)
file(GLOB_RECURSE MYMOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)
file(GLOB_RECURSE TESTSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS test/*.cpp)

#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${MYICONS})
QT5_WRAP_UI(QT_FORMS_HPP ${MYFORMS})
#QT5_WRAP_CPP(QT_MOC_HPP ${MYMOC}) 

add_executable(${PROJECT_NAME}
   ${MYSOURCE}
   ${MYICONS}
   ${MYOBJ}
   ${MYMOC}
   ${QT_FORMS_HPP}
   )

target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    Qt5::Widgets
    Qt5::Gui
    Qt5::Core
    )

install(TARGETS fixedwing_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#if(${GTEST_FOUND})
#    set(TEST_NAME ${PROJECT_NAME}_UnitTest)
#    include_directories(
#        include
#        ${GTEST_INCLUDE_DIRS}
#        )
#    add_executable(TEST_NAME
#        TESTSOURCE
#        )
#    target_link_libraries(${TEST_NAME}
#        ${GTEST_LIBRARIES}
#        ${CATKIN_LIBRARIES}
#        gtest
#        gtest_main
#        pthread
#        )
#endif(${GTEST_FOUND})
