#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <April2Pose.h>


April2Pose::April2Pose(int _id){
    ids.push_back(_id);
}

April2Pose::April2Pose(std::vector<int> _ids){
    for(int i=0; i<_ids.size(); i++
        // for(std::vector<int>::const_iterator it = _ids.begin();
        // it != _ids.end();
        // ++it
        ) {
            // ids.push_back(*it);
            ids.push_back(_ids[i]);
        }
    ROS_INFO("I am searching for %lu ids", ids.size());
}

void April2Pose::aprilCallback(const apriltag_ros::AprilTagDetectionArray& msg){
    for (int i=0; i<msg.detections.size(); i++) {
        apriltag_ros::AprilTagDetection the_detection = msg.detections[i];
        ROS_INFO("I heard: [%d]", the_detection.id[0]);
        ROS_INFO("Position x: [%f]", the_detection.pose.pose.pose.position.x);
        ROS_INFO("Position y: [%f]", the_detection.pose.pose.pose.position.y);
        ROS_INFO("Position z: [%f]", the_detection.pose.pose.pose.position.z);
        ROS_INFO("Orientation w: [%f]", the_detection.pose.pose.pose.orientation.w);
        ROS_INFO("Orientation x: [%f]", the_detection.pose.pose.pose.orientation.x);
        ROS_INFO("Orientation y: [%f]", the_detection.pose.pose.pose.orientation.y);
        ROS_INFO("Orientation z: [%f]", the_detection.pose.pose.pose.orientation.z);
        
        for(int i=0; i<ids.size(); i++
                // std::vector<int>::const_iterator it = ids.begin();
                // it != ids.end();
                // it++
                ) {
            // ROS_INFO("Sto controllando %d", *it);
            // if(*it == the_detection.id[0]){
                // ROS_INFO("Era quello giusto!!!! %d", *it);
            // }
            ROS_INFO("Sto controllando %d", ids[i]);
            if(ids[i] == the_detection.id[0]){
                ROS_INFO("Era quello giusto!!!! %d", ids[i]);
            }
        }
    }
    ROS_INFO("Done aprilCallback");
}

