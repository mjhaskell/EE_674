cmake_minimum_required(VERSION 3.4)
project(fixedwing_sim)

#IF (NOT TARGET gtest)
#    # Prevent GoogleTest from overriding our compiler/linker options
#    # when building with Visual Studio
#    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#    add_subdirectory(libs/googletest)
#ENDIF()

find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
#find_package(Eigen3 REQUIRED)

get_target_property(QtCore_location Qt5::Core LOCATION)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)
include_directories(include)

catkin_package()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE MYSOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB MYFORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB MYRESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} icons/*.qrc)
file(GLOB_RECURSE MYMOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)

#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${MYRESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${MYFORMS})
#QT5_WRAP_CPP(QT_MOC_HPP ${MYMOC})

set(TEST_NAME ${PROJECT_NAME}_UnitTest)

#set( MYRESOURCES resources/images.qrc)


add_executable(${PROJECT_NAME}
   ${MYSOURCE}
   ${MYRESOURCES}
   ${MYMOC}
   ${QT_FORMS_HPP}
   )

target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${eigen3_LIBRARIES}
    Qt5::Widgets
    Qt5::Gui
    Qt5::Core
    )

install(TARGETS fixedwing_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#set(MYFILES
##    ../qt_planner/src/main.cpp
#    ../qt_planner/src/obstaclemap.cpp
#    ../qt_planner/include/qt_planner/obstaclemap.hpp
#    ../qt_planner/src/transforms2d.cpp
#    ../qt_planner/include/qt_planner/transforms2d.hpp
##    ../qt_planner/include/qt_planner/widget.hpp
##    ../qt_planner/src/widget.cpp
##    ../qt_planner/src/helper.cpp
##    ../qt_planner/include/qt_planner/helper.hpp
##    ../qt_planner/src/qnode.cpp
##    ../qt_planner/include/qt_planner/qnode.hpp
#    )
#
#add_executable(${TEST_NAME}
#    ../qt_planner/tests/map_unittests.cpp
#    ../qt_planner/tests/transforms2d_unittests.cpp
#    ${MYFILES}
#    )
#
#target_link_libraries(${TEST_NAME}
#    gtest
#    gtest_main
#    )

