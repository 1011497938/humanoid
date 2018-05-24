cmake_minimum_required(VERSION 2.8.3)
project(dcalibrator)
#LIST(APPEND CMAKE_MODULE_PATH ${dvision_SOURCE_DIR}/cmake_modules)

SET(ZJUDANCER_GUI $ENV{ZJUDANCER_GUI})
if (ZJUDANCER_GUI EQUAL 1)
  add_definitions(-std=c++11 -Wall  -Wextra -Wfloat-equal -Wno-unused-result) #-Weffc++ -pedantic

  find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    dvision
    ros_h264_streamer
  )

  find_package(OpenCV 3 REQUIRED)
  catkin_package(
  #  INCLUDE_DIRS include
    #  LIBRARIES dmotion
    #  CATKIN_DEPENDS dprocess
    #  DEPENDS system_lib
  )

  find_package(Qt5 COMPONENTS
      Core
      Widgets
      REQUIRED)

  set(CMAKE_INCLUDE_CURRENT_DIR on)

  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)

  qt5_wrap_ui(UI_SRC src/mainwindow.ui src/undist.ui)

  include_directories(include src ${catkin_INCLUDE_DIRS})

  add_executable(dcalibrator src/main.cpp
      src/mainwindow.cpp
      src/mymodel.cpp
      #src/dvis.qrc
      src/clabel.cpp
      src/undist.cpp
      ${UI_SRC}
      )


  target_link_libraries(dcalibrator
      ${catkin_LIBRARIES}
      ${OpenCV_LIBS}
      Qt5::Core Qt5::Widgets)
endif()
