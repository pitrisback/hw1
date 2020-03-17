#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <April2Pose.h>
#include "ros/package.h"
#include <fstream>

April2Pose::April2Pose(int _id){
    ids.push_back(_id);
}

April2Pose::April2Pose(std::vector<int> _ids){
    for(int i=0; i<_ids.size(); i++) {
        ids.push_back(_ids[i]);
    }
    ROS_INFO("I am searching for %lu ids", ids.size());
}

void April2Pose::aprilCallback(const apriltag_ros::AprilTagDetectionArray& msg){
    for (int i=0; i<msg.detections.size(); i++) {
        apriltag_ros::AprilTagDetection the_detection = msg.detections[i];
        ROS_INFO("I heard: [%d]", the_detection.id[0]);
        
        for(int i=0; i<ids.size(); i++) {
            ROS_INFO("Sto controllando %d", ids[i]);
            if(ids[i] == the_detection.id[0]){
                ROS_INFO("Era quello giusto!!!! %d", ids[i]);
                ROS_INFO("Position x: [%f]", the_detection.pose.pose.pose.position.x);
                ROS_INFO("Position y: [%f]", the_detection.pose.pose.pose.position.y);
                ROS_INFO("Position z: [%f]", the_detection.pose.pose.pose.position.z);
                ROS_INFO("Orientation w: [%f]", the_detection.pose.pose.pose.orientation.w);
                ROS_INFO("Orientation x: [%f]", the_detection.pose.pose.pose.orientation.x);
                ROS_INFO("Orientation y: [%f]", the_detection.pose.pose.pose.orientation.y);
                ROS_INFO("Orientation z: [%f]", the_detection.pose.pose.pose.orientation.z);

                std::stringstream ss;
                ss << ros::package::getPath("hw1") << "/output/pose_" << ids[i] << ".txt";
                std::ofstream file_out;
                ROS_INFO("File name %s", ss.str().c_str());
                file_out.open(ss.str().c_str());
                file_out << "Pose and orientation for tag " << ids[i] << std::endl;
                file_out << "Position x: " << the_detection.pose.pose.pose.position.x << std::endl;
                file_out << "Position y: " << the_detection.pose.pose.pose.position.y << std::endl;
                file_out << "Position z: " << the_detection.pose.pose.pose.position.z << std::endl;
                file_out << "Orientation w: " << the_detection.pose.pose.pose.orientation.w << std::endl;
                file_out << "Orientation x: " << the_detection.pose.pose.pose.orientation.x << std::endl;
                file_out << "Orientation y: " << the_detection.pose.pose.pose.orientation.y << std::endl;
                file_out << "Orientation z: " << the_detection.pose.pose.pose.orientation.z << std::endl;
                file_out.close();
            }
        }
    }
    ROS_INFO("Done aprilCallback");
}

