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

void test_pcl2pose() {
    std::string topic = "camera/depth/points";
    // Kinect topic
    // https://answers.ros.org/question/10038/getting-point-cloud-data-from-the-kinect/
    // using regiseted is probably a good idea
    // https://homes.cs.washington.edu/~edzhang/tutorials/kinect2/kinect3.html
    // std::string topic = "head_mount_kinect/depth_registered/points";
    // std::string topic = "camera/depth_registered/points";
    PCL2Pose parser = PCL2Pose(topic);
}

int main(int argc, char** argv)
{
    ROS_INFO("Booting pcl_find_objects");
    ros::init(argc, argv, "sub_pcl");

    test_pcl2pose();

    ROS_INFO("Spinning");
    ros::spin();
}
