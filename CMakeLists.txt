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
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
)

# things for PCL
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###### APRILTAG ######

add_executable(april_parse
    src/april_parse.cpp
)

# add_dependencies(april_parse hw1_generate_messages_cpp)
target_link_libraries(april_parse
    ${catkin_LIBRARIES}
)

###### PCL POSE ######

# pcl_find_objects needs this files
add_executable(pcl_find_objects
    src/pcl_find_objects.cpp
)

# pcl_find_objects needs this libraries
target_link_libraries(pcl_find_objects
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

###### PCL SAVE ######

# pcl_find_objects needs this files
add_executable(pcl_save
    src/pcl_save.cpp
)

# pcl_find_objects needs this libraries
target_link_libraries(pcl_save
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

###### PCL VIZ ######

# pcl_find_objects needs this files
add_executable(pcl_viz
    src/pcl_viz.cpp
)

# pcl_find_objects needs this libraries
target_link_libraries(pcl_viz
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
