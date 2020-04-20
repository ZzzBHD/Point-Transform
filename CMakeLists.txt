cmake_minimum_required(VERSION 2.8.3)
# project(rosbag_to_pcd)
project(pointcloud_transfer)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge 
  image_transport
  pcl_conversions 
  rospy
)
find_package( OpenCV REQUIRED )
find_package( PCL 1.3 REQUIRED )
find_package(Boost REQUIRED COMPONENTS
thread)
## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}${OpenCV_INCLUDE_DIRS})
include_directories(
  include ${PCL_INCLUDE_DIRS})
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
  src)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(tranc_c_node 
  src/tranc_c_node.cpp
  src/trans_class.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(tranc_pointcloud_node ${catkin_LIBRARIES})
target_link_libraries(tranc_c_node  ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(tranc_c_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(pubc_node
  src/pubc_node.cpp
  src/trans_class.cpp
)
add_dependencies(pubc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pubc_node
  ${catkin_LIBRARIES}
)

add_executable(tranc_p2c_node
  src/tranc_p2c_node.cpp
  src/trans_class.cpp
)
add_dependencies(tranc_p2c_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tranc_p2c_node
  ${catkin_LIBRARIES}
)