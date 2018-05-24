cmake_minimum_required(VERSION 2.8.3)
project(dmonitor)

SET(ZJUDANCER_GUI $ENV{ZJUDANCER_GUI})
message("${ZJUDANCER_GUI}")
if (ZJUDANCER_GUI EQUAL 1)
add_definitions(-std=c++11 -Wall -g -O2  -Wextra -Wfloat-equal -Wno-unused-result) #-Weffc++ -pedantic

find_package(catkin REQUIRED
  COMPONENTS 
  roscpp
  dvision
  dtransmit
  dconfig
  dmotion
  ros_h264_streamer
)

catkin_package(
#  INCLUDE_DIRS include
  #  LIBRARIES dmotion
  #  CATKIN_DEPENDS dprocess
  #  DEPENDS system_lib
)

find_package(Qt5 COMPONENTS
    Core
    Widgets
    Qml
    Quick
    REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR on)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(include src ${catkin_INCLUDE_DIRS})

add_executable(dmonitor
    src/main.cpp
#    src/robot.cpp
#    src/field.cpp
#    src/ball.cpp
#    src/baseObject.cpp
#    src/guiImage.cpp
#    src/viewRange.cpp
    src/map.cpp
#    include/dmonitor/robot.hpp
#    include/dmonitor/field.hpp
#    include/dmonitor/ball.hpp
#    include/dmonitor/baseObject.hpp
#    include/dmonitor/guiImage.hpp
#    include/dmonitor/utils.hpp
    include/dmonitor/map.hpp
#    include/dmonitor/viewRange.hpp
    src/qml.qrc
    src/qml/main.qml
#    src/qml/Robot.qml
#    src/qml/Ball.qml
#    src/qml/Box.qml
#    src/qml/ViewRange.qml
#    src/qml/Selection.qml
#    src/js/componentCreation.js
    )


target_link_libraries(dmonitor
    ${catkin_LIBRARIES}
    Qt5::Qml
    Qt5::Quick
    Qt5::Core
    Qt5::Widgets)

endif()
