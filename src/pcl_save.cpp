#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

// typedef pcl::PointCloud<pcl::PointXYZ> T_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> T_PointCloud;

void callback_main(const T_PointCloud::ConstPtr& msg) {
    // BAD practice high technical debt!
    // wish you had used classes now huh
    // std::string pcl_data_file = "../pcl_data/pcl_kinect.pcd";
    // std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_kinect.pcd";
    // std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_kinect_3obj.pcd";
    std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_rgb_3obj.pcd";

    ROS_INFO("Saving cloud: width = %d, height = %d", msg->width, msg->height);
    pcl::io::savePCDFileASCII(pcl_data_file, *msg);
}


int main(int argc, char** argv)
{
    ROS_INFO("Booting pcl_save");

    ros::init(argc, argv, "sub_pcl_save");
    ros::NodeHandle nh;

    std::string cloud_topic = "camera/depth_registered/points";
    ros::Subscriber sub = nh.subscribe<T_PointCloud>(
            cloud_topic,
            10,
            callback_main
        );

    // ROS_INFO("Spinning once");
    // ros::spinOnce();
    ROS_INFO("Spinning");
    ros::spin();
}
