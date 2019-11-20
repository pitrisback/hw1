#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

void chatterCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
    // documentation on msg structure
    // http://docs.ros.org/kinetic/api/apriltag_ros/html/msg/AprilTagDetectionArray.html
    // and detection object
    // http://docs.ros.org/kinetic/api/apriltag_ros/html/msg/AprilTagDetection.html
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
