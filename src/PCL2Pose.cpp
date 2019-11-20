#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <PCL2Pose.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PCL2Pose::PCL2Pose(std::string _cloud_topic) {
    cloud_topic = _cloud_topic;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>(
            cloud_topic,
            1,
            p2p_callback
        );
    ROS_INFO("Done setting up PCL2Pose");
}

void PCL2Pose::p2p_callback(const PointCloud::ConstPtr& msg) {
    ROS_INFO("Cloud: width = %d, height = %d", msg->width, msg->height);
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}
