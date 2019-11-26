#ifndef APRIL2POSE_H
#define APRIL2POSE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

class April2Pose {
    public:
        April2Pose(int _id);
        April2Pose(std::vector<int> _ids);
        void aprilCallback(const apriltag_ros::AprilTagDetectionArray& msg);
    protected: 
        std::vector<int> ids; 
};

#endif