cmake_minimum_required(VERSION 2.8.3)
project(dvision)

set(ZJUDANCER_GUI $ENV{ZJUDANCER_GUI})
# add_definitions(-std=c++11 -Wall -g) # -g -Wextra -Wno-unused-result) #-Wfloat-equal -Weffc++ -pedantic
add_definitions(-std=c++11 -Wall -O2) # -g -Wextra -Wno-unused-result) #-Wfloat-equal -Weffc++ -pedantic

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  dmsgs
  dmotion
  dprocess
  dtransmit
  dbehavior
  dconfig
  ros_h264_streamer
  cv_bridge
  darknetcxx)

# find_package(SuiteSparse REQUIRED)
find_package(OpenCV REQUIRED)
message("Using OpenCV: ${OpenCV_INCLUDE_DIRS}")

find_package(Eigen3 REQUIRED)
# find_package(G2O REQUIRED)

find_package(Boost REQUIRED)
message("Boost FOUND: >>>>>>>>>>>>>>>>>>>>> ${Boost_INCLUDE_DIRS} ${Boost_LIBRARIES} ${Boost_LIBRARY_DIRS}")

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
    LIBRARIES dvisionlib ${OpenCV_LIBS}
  #  CATKIN_DEPENDS dprocess
  #  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}) #${CSPARSE_INCLUDE_DIR}) #${Boost_INCLUDE_DIRS})

#SET(G2O_LIBS
    #${G2O_CORE_LIBRARY}
    #${G2O_STUFF_LIBRARY}
    #${G2O_CLI_LIBRARY}
    #${G2O_SOLVER_CHOLMOD}
    #${G2O_SOLVER_CSPARSE}
    #${G2O_SOLVER_CSPARSE_EXTENSION}
    #${G2O_SOLVER_DENSE}
    #${G2O_SOLVER_PCG}
    #${G2O_SOLVER_SLAM2D_LINEAR}
    #${G2O_SOLVER_STRUCTURE_ONLY}
    #${G2O_SOLVER_EIGEN}
    #${G2O_TYPES_DATA}
    #${G2O_TYPES_ICP}
    #${G2O_TYPES_SBA}
    #${G2O_TYPES_SCLAM2D}
    #${G2O_TYPES_SIM3}
    #${G2O_TYPES_SLAM2D}
    #${G2O_TYPES_SLAM3D}
    #)


add_library(dvisionlib
  src/dvision.cpp
  src/frame.cpp
  src/parameters.cpp
  src/camera/camera.cpp
  src/camera/camera_dummy.cpp
  src/camera/distortionModel.cpp
  src/camera/ipm.cpp
  src/camera/projection.cpp
  src/detector/object_detector.cpp
  src/detector/ball_detector.cpp
  src/detector/circle_detector.cpp
  src/detector/field_detector.cpp
  src/detector/goal_detector.cpp
  src/detector/line_detector.cpp
  src/detector/obstacle_detector.cpp
  src/detector/line_classifier.cpp
  src/localization/amcl/amcl.cpp
  src/localization/amcl/pose.cpp
  src/localization/amcl/particle.cpp
  src/localization/amcl/map.cpp
  src/localization/amcl/kdtree.cpp
  src/tools/line_segment.cpp
  src/tools/utils.cpp
  src/tools/kalman.cpp
  src/tools/ball_tracker.cpp
  )

target_link_libraries(dvisionlib ${catkin_LIBRARIES} ${OpenCV_LIBS}) #${CSPARSE_LIBRARY} ${G2O_LIBS}) # ${Boost_LIBRARIES})

add_executable(dvision_node src/main.cpp)
target_link_libraries(dvision_node dvisionlib ${catkin_LIBRARIES})


if(ZJUDANCER_GUI EQUAL 1)
add_executable(capture src/tools/capture.cpp)
target_link_libraries(capture dvisionlib)

add_executable(calib src/camera/calibration.cpp)
target_link_libraries(calib ${OpenCV_LIBS})

#add_executable(cvcalib src/camera/offical-calibration.cpp)
#target_link_libraries(cvcalib ${OpenCV_LIBS})
endif()

#add_executable(videowriter test/test_videowriter.cpp)
#target_link_libraries(videowriter dvisionlib ${Opencv_LIBS})

#add_executable(test_frame test/test_frame.cpp)
#target_link_libraries(test_frame dvisionlib ${OpenCV_LIBS})
#
#add_executable(test_frameSend test/test_frameSend.cpp)
#target_link_libraries(test_frameSend dvisionlib ${OpenCV_LIBS})
#
#add_executable(test_frameRecv test/test_frameRecv.cpp)
#target_link_libraries(test_frameRecv dvisionlib ${OpenCV_LIBS})
### TEST

catkin_add_gtest(test_amcl test/test_amcl.cpp)
target_link_libraries(test_amcl dvisionlib)
target_link_libraries(test_amcl ${catkin_LIBRARIES})

catkin_add_gtest(test_cameraDummy test/test_cameraDummy.cpp)
target_link_libraries(test_cameraDummy dvisionlib)
target_link_libraries(test_cameraDummy ${catkin_LIBRARIES})

catkin_add_gtest(test_distortionModel test/test_distortionModel.cpp)
target_link_libraries(test_distortionModel dvisionlib)
target_link_libraries(test_distortionModel ${catkin_LIBRARIES})

catkin_add_gtest(test_projection test/test_projection.cpp)
target_link_libraries(test_projection dvisionlib)
target_link_libraries(test_projection ${catkin_LIBRARIES})

catkin_add_gtest(test_localization test/test_localization.cpp)
target_link_libraries(test_localization dvisionlib)
target_link_libraries(test_localization ${catkin_LIBRARIES})

catkin_add_gtest(test_ringbuffer test/test_ringbuffer.cpp)
#target_link_libraries(test_ringbuffer dvisionlib)
target_link_libraries(test_ringbuffer ${catkin_LIBRARIES})
