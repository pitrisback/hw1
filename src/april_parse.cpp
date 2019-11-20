#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

void chatterCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
    if (msg.detections.size() > 0) {
        apriltag_ros::AprilTagDetection first_detection = msg.detections[0];
        ROS_INFO("I heard: [%d]", first_detection.id[0]);
    } else {
        ROS_INFO("No tag found");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "april_parse_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("tag_detections", 1000, chatterCallback);
    ros::spin();
    return 0;
}
