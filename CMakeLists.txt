cmake_minimum_required(VERSION 3.0.2)
project(rotema)

## Temukan paket catkin dan komponen yang diperlukan
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
)

## Temukan pustaka OpenCV
find_package(OpenCV REQUIRED)

## Deklarasikan paket catkin
catkin_package()

## Tambahkan direktori header
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Tambahkan eksekusi untuk RobotNavigationNode
add_executable(RobotNavigationNode src/RobotNavigation.cpp)
target_link_libraries(RobotNavigationNode
  ${catkin_LIBRARIES}
)

## Tambahkan eksekusi untuk RobotPositionNode
add_executable(RobotPositionNode src/RobotPosition.cpp)
target_link_libraries(RobotPositionNode
  ${catkin_LIBRARIES}
)

## Tambahkan eksekusi untuk RobotVelocityCommandNode
add_executable(RobotVelocityCommandNode src/RobotVelocityCommand.cpp)
target_link_libraries(RobotVelocityCommandNode
  ${catkin_LIBRARIES}
)

## Tambahkan eksekusi untuk CameraCalibrationNode dengan OpenCV
add_executable(CameraCalibrationNode src/CameraCalibration.cpp)
target_link_libraries(CameraCalibrationNode
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
