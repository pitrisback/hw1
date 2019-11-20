#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

// tutorials
// http://wiki.ros.org/pcl_ros
// http://wiki.ros.org/pcl/Tutorials

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// pcl::PointCloud

void callback(const PointCloud::ConstPtr& msg)
{
    ROS_INFO("Cloud: width = %d, height = %d", msg->width, msg->height);
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
    ROS_INFO("Booting Subscriber");
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    // std::string topic = "head_mount_kinect/depth_registered/points";
    // std::string topic = "camera/depth_registered/points";
    std::string topic = "camera/depth/points";
    ros::Subscriber sub = nh.subscribe<PointCloud>(topic, 1, callback);
    ros::spin();
}
