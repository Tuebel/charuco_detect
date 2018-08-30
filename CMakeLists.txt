cmake_minimum_required(VERSION 2.8.3)
project(charuco_detect)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf2_ros
    geometry_msgs
    cv_bridge 
    image_transport
    tf2_geometry_msgs)
find_package(OpenCV REQUIRED aruco)

# packaging
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES charuco_detect
  CATKIN_DEPENDS
    roscpp
    tf2_ros
    geometry_msgs
    cv_bridge 
    image_transport
    tf2_geometry_msgs
  DEPENDS OpenCV)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# libraries
add_library(aruco_detect)
target_sources(aruco_detect PRIVATE
  src/aruco/aruco_detect.cpp)
target_link_libraries(aruco_detect PUBLIC
  ${OpenCV_LIBS})

add_library(charuco_detect)
target_sources(charuco_detect PRIVATE
  src/charuco/board.cpp
  src/charuco/charuco_detect.cpp)
target_link_libraries(charuco_detect PUBLIC
  aruco_detect
  ${OpenCV_LIBS})

add_library(charuco_conversion)
target_sources(charuco_conversion PRIVATE 
  src/conversion/camera_conversion.cpp
  src/conversion/pose_conversion.cpp)
target_link_libraries(charuco_conversion PUBLIC
  ${OpenCV_LIBS})

add_library(charuco_visualization)
target_sources(charuco_visualization PRIVATE 
  src/visualization/draw_markers.cpp
  src/visualization/draw_pose.cpp)
target_link_libraries(charuco_detect PUBLIC
  aruco_detect)

# executables
add_executable(aruco_pose_node 
    src/nodes/aruco_pose_node.cpp)
target_link_libraries(aruco_pose_node
  ${catkin_LIBRARIES}
  charuco_conversion
  aruco_detect
  charuco_visualization)

add_executable(charuco_detect_node 
    src/nodes/charuco_detect_node.cpp)
target_link_libraries(charuco_detect_node
  ${catkin_LIBRARIES}
  charuco_conversion
  charuco_detect
  charuco_visualization)

add_executable(write_default_board
    src/nodes/write_default_board.cpp)
target_link_libraries(write_default_board 
  ${catkin_LIBRARIES}
  charuco_detect)

# Mark executables and/or libraries for installation
install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
