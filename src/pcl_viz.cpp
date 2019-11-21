#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> T_PointCloud;


void show_cloud(T_PointCloud::Ptr & cloud,
                std::string cloud_name,
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler
        ) {
    std::string viewer_name = "3D Viewer";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
            pcl::visualization::PCLVisualizer (viewer_name));

    // Define R,G,B colors for the point cloud
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          // cloud_color_handler (cloud, 255, 255, 255);
  
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, cloud_name);
    // bool addPointCloud(
    //      const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    //      const PointCloudColorHandler<PointT> &color_handler,
    //      const std::string &id = "cloud",
    //      int viewport = 0 )

    viewer->initCameraParameters ();
    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped ())
        viewer->spinOnce ( 1 );
}

void filter_cloud(T_PointCloud::Ptr & cloud,
        std::string axis,
        float lower_bound,
        float upper_bound
        ) {
    // the axis are ordered XYB <-> RGB

    ROS_INFO("Filtering along %s in range (%f, %f)", axis.c_str(), lower_bound, upper_bound);

    pcl::PassThrough<pcl::PointXYZ> pass_through;
    pass_through.setInputCloud (cloud);
    // pass_through.setFilterLimits (0.0, 0.5);
    // pass_through.setFilterFieldName ("z");
    pass_through.setFilterLimits (lower_bound, upper_bound);
    pass_through.setFilterFieldName (axis);
    pass_through.filter( *cloud );
}

int main(int argc, char** argv)
{
    ROS_INFO("Booting pcl_viz");

    // ros::init(argc, argv, "sub_pcl_viz");
    // ros::NodeHandle nh;

    std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_kinect.pcd";

    T_PointCloud::Ptr cloud(new T_PointCloud);
    pcl::io::loadPCDFile(pcl_data_file, *cloud);

    ROS_INFO("Loaded cloud: width = %d, height = %d", cloud->width, cloud->height);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          orig_cloud_color_handler (cloud, 255, 255, 255);
    // show_cloud(cloud, "Original", orig_cloud_color_handler);

    // filter along z to keep only the table
    // filter_cloud(cloud, "z", 1.5, 2.0); // good values
    // filter_cloud(cloud, "y", -0.5, 0.3); // good values
    // filter_cloud(cloud, "x", -0.5, 0.5); // good values

    ROS_INFO("CLI: Filtering in range (%s, %s)", argv[1], argv[2]);
    float lower_bound = std::atof(argv[1]);
    float upper_bound = std::atof(argv[2]);
    filter_cloud(cloud, "z", lower_bound, upper_bound);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          z_filt_color_handler (cloud, 0, 180, 255);
    // show_cloud(cloud, "Filtered z", z_filt_color_handler);

    ROS_INFO("CLI: Filtering in range (%s, %s)", argv[3], argv[4]);
    lower_bound = std::atof(argv[3]);
    upper_bound = std::atof(argv[4]);
    filter_cloud(cloud, "y", lower_bound, upper_bound);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          y_filt_color_handler (cloud, 180, 0, 255);
    // show_cloud(cloud, "Filtered y", y_filt_color_handler);

    ROS_INFO("CLI: Filtering in range (%s, %s)", argv[5], argv[6]);
    lower_bound = std::atof(argv[5]);
    upper_bound = std::atof(argv[6]);
    filter_cloud(cloud, "x", lower_bound, upper_bound);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          x_filt_color_handler (cloud, 180, 0, 255);
    show_cloud(cloud, "Filtered x", x_filt_color_handler);
}

