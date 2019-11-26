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
    protected: 
        std::vector<int> ids; 
        ros::NodeHandle nh;
        ros::Subscriber sub;
        void Subscribe();
};

#endif