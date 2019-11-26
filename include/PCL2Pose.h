#ifndef PCL2POSE_H
#define PCL2POSE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PCL2Pose {
    public:
        PCL2Pose(std::string _cloud_topic);

    protected:
        std::string cloud_topic;
        static void p2p_callback(const PointCloud::ConstPtr& msg);
};

#endif
