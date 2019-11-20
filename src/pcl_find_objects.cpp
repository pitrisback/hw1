#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <PCL2Pose.h>

// tutorials
// http://wiki.ros.org/pcl_ros
// http://wiki.ros.org/pcl/Tutorials

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback_main(const PointCloud::ConstPtr& msg) {
    ROS_INFO("Cloud: width = %d, height = %d", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }
}

int main(int argc, char** argv)
{
    ROS_INFO("Booting pcl_find_objects");

    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;

    std::string cloud_topic = "camera/depth/points";
    // std::string cloud_topic = "head_mount_kinect/depth_registered/points";
    // std::string cloud_topic = "camera/depth_registered/points";
    ros::Subscriber sub = nh.subscribe<PointCloud>(
            cloud_topic,
            1,
            callback_main
        );

    ROS_INFO("Spinning");
    ros::spin();
}
