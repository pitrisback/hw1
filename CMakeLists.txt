cmake_minimum_required(VERSION 2.8.3)
project(hw1)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    apriltag_ros
)

# magic pcl link
# https://answers.ros.org/question/123261/unable-to-include-pcl-files-in-header/
# actual documentation
# http://www.pointclouds.org/documentation/tutorials/using_pcl_pcl_config.php
find_package(PCL REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
        apriltag_ros
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(april_parse src/april_parse.cpp)
# add_dependencies(april_parse hw1_generate_messages_cpp)
target_link_libraries(april_parse ${catkin_LIBRARIES})

add_executable(pcl_find_objects src/pcl_find_objects.cpp)
target_link_libraries(pcl_find_objects ${PCL_LIBRARIES} ${catkin_LIBRARIES})
