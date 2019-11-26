#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

// #include <pcl/impl/point_types.hpp>

#include <PCL2Pose.h>

// tutorials
// http://wiki.ros.org/pcl_ros
// http://wiki.ros.org/pcl/Tutorials

typedef pcl::PointCloud<pcl::PointXYZ> T_PointCloud;
// typedef pcl::PointCloud<pcl::PointXYZRGB> T_PointCloud;

void callback_main(const T_PointCloud::ConstPtr& msg) {
    ROS_INFO("Cloud: width = %d, height = %d", msg->width, msg->height);

    ROS_INFO("\tPoint %d: (%f, %f, %f)",
            0,
            msg->points[0].x,
            msg->points[0].y,
            msg->points[0].z
            // msg->points[0].rgb
        );
    pcl::PointXYZ pt = msg->points[0];
    // pcl::PointXYZRGB pt = msg->points[0];
    ROS_INFO("\t(%f, %f, %f)", pt.x, pt.y, pt.z);

    // for (std::size_t i = 0; i < msg->points.size(); ++i)
    // {
        // ROS_INFO("\tPoint %lu: (%f, %f, %f)",
                // i,
                // msg->points[i].x,
                // msg->points[i].y,
                // msg->points[i].z
                // // msg->points[i].rgb
            // );
    // }

    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
    // BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points) {
        ROS_INFO("\t(%f, %f, %f)", pt.x, pt.y, pt.z);
        break;
    }

    const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > data = msg->points;
    ROS_INFO("Cloud has  size %lu", data.size());
    pt = data[0];
    ROS_INFO("\t(%f, %f, %f)", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
    ROS_INFO("Booting pcl_find_objects");

    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;

    pcl::PointXYZ punto;

    // std::string cloud_topic = "camera/depth/points";
    // std::string cloud_topic = "head_mount_kinect/depth_registered/points";
    std::string cloud_topic = "camera/depth_registered/points";
    ros::Subscriber sub = nh.subscribe<T_PointCloud>(
            cloud_topic,
            1,
            callback_main
        );

    ROS_INFO("Spinning");
    ros::spin();
}
