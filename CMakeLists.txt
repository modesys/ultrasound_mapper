cmake_minimum_required(VERSION 2.8.3)
project(map_ros)

add_compile_options(-std=c++11)
find_package(octomap REQUIRED)
find_package( OpenCV REQUIRED )
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    pcl_ros
    pcl_conversions
    geometry_msgs
    nav_msgs
    grid_map_core
    grid_map_ros
    grid_map_cv
    grid_map_filters
    grid_map_loader
    grid_map_msgs
    grid_map_octomap
    grid_map_rviz_plugin
    grid_map_visualization
    cv_bridge
    octomap_msgs
    filters
    image_transport
    )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${OpenCV_LIBRARIES}
  CATKIN_DEPENDS
      roscpp
      sensor_msgs
      std_msgs
      message_generation
      pcl_ros
      pcl_conversions
      geometry_msgs
      nav_msgs
      grid_map_core
      grid_map_ros
      grid_map_cv
      grid_map_filters
      grid_map_loader
      grid_map_msgs
      grid_map_octomap
      grid_map_rviz_plugin
      grid_map_visualization
      octomap_msgs
      filters
      DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
add_executable(map_test src/map_test.cpp ${SRCS})
target_link_libraries(map_test ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(gridMapLoader src/gridMapLoader.cpp ${SRCS})
target_link_libraries(gridMapLoader ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(imageConverter src/imageConverter.cpp ${SRCS})
target_link_libraries(imageConverter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


include_directories(${catkin_INCLUDE_DIRS})
add_executable(pointcloud_reader_node include/cloud.cpp
                                       src/pointcloud_reader_node.cpp ${SRCS})
target_link_libraries(pointcloud_reader_node
    ${catkin_LIBRARIES})


