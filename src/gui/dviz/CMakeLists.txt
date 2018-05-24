cmake_minimum_required(VERSION 2.8.3)
project(dviz)

set(ZJUDANCER_GUI $ENV{ZJUDANCER_GUI})

if (ZJUDANCER_GUI EQUAL 1)

add_compile_options(-std=c++11 -g)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    dmsgs
    dtransmit
    dvision
    dconfig)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dviz
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(Qt5 COMPONENTS
    Core
    Widgets
    REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/view.cpp
  src/render.cpp
  src/mainwindow.cpp
  src/model.cpp
  src/common.cpp
  src/control_widget.cpp
  src/streamer.cpp
  # QGraphicsItems
  src/items/item_manager.cpp
  src/items/item.cpp
  src/items/field.cpp
  src/items/ball.cpp
  src/items/robot.cpp
  src/items/viewrange.cpp
  src/items/whitelines.cpp
  src/items/whitepoints.cpp
  src/items/particles.cpp
  src/items/world.cpp
  src/items/circle.cpp
  src/items/goals.cpp
  src/items/obstacles.cpp
  src/items/dest.cpp

 ########
  include/view.hpp
  include/render.hpp
  include/mainwindow.hpp
  include/model.hpp
  include/common.hpp
  include/control_widget.hpp
  include/streamer.hpp
  include/item_manager.hpp # without this, vtable ...
  include/item.hpp
  include/field.hpp
  include/ball.hpp
  include/robot.hpp
  include/viewrange.hpp
  include/whitelines.hpp
  include/whitepoints.hpp
  include/particles.hpp
  include/world.hpp
  include/circle.hpp
  include/goals.hpp
  include/obstacles.hpp
  include/dest.hpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets)

endif()
