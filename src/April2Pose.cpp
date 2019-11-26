#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <April2Pose.h>


April2Pose::April2Pose(int _id){
    ids.push_back(_id);
}

April2Pose::April2Pose(std::vector<int> _ids){
    for(std::vector<int>::const_iterator it = _ids.begin();
        it != _ids.end();
        ++it ) {
            ids.push_back(*it);
        }
    ROS_INFO("I am searching for %d ids", ids.size());
}

void April2Pose::aprilCallback(const apriltag_ros::AprilTagDetectionArray& msg){
    if (msg.detections.size() > 0) {
        apriltag_ros::AprilTagDetection first_detection = msg.detections[0];
        ROS_INFO("I heard: [%d]", first_detection.id[0]);
        ROS_INFO("Position x: [%f]", first_detection.pose.pose.pose.position.x);
        ROS_INFO("Position y: [%f]", first_detection.pose.pose.pose.position.y);
        ROS_INFO("Position z: [%f]", first_detection.pose.pose.pose.position.z);
        ROS_INFO("Orientation w: [%f]", first_detection.pose.pose.pose.orientation.w);
        ROS_INFO("Orientation x: [%f]", first_detection.pose.pose.pose.orientation.x);
        ROS_INFO("Orientation y: [%f]", first_detection.pose.pose.pose.orientation.y);
        ROS_INFO("Orientation z: [%f]", first_detection.pose.pose.pose.orientation.z);
        
        for(std::vector<int>::const_iterator it = ids.begin();
        it != ids.end();
        ++it ) {
            if(*it == first_detection.id[0]){
                ROS_INFO("Era quello giusto!!!! %d", *it);
            }
        }

    } else {
        ROS_INFO("No tag found");
    }
}

