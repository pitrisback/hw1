#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> T_PointCloud;

void show_cloud(T_PointCloud::Ptr & cloud,
                std::string cloud_name,
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler
        ) {
    std::string viewer_name = "3D Viewer";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
            pcl::visualization::PCLVisualizer (viewer_name));
  
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, cloud_name);
    // prototype used:
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
    // the axis are colored in order XYB <-> RGB

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

    // ############# load the cloud #############

    // std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_kinect.pcd";
    std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_test_from_mesh.pcd";

    T_PointCloud::Ptr cloud(new T_PointCloud);
    pcl::io::loadPCDFile(pcl_data_file, *cloud);

    ROS_INFO("Loaded cloud: width = %d, height = %d", cloud->width, cloud->height);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          orig_cloud_color_handler (cloud, 255, 255, 255);
    show_cloud(cloud, "Original", orig_cloud_color_handler);
    return 0;

    // ############# chop the cloud #############

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
    // show_cloud(cloud, "Filtered x", x_filt_color_handler);

    // ############# find the plane #############
    // http://pointclouds.org/documentation/tutorials/planar_segmentation.php

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    // seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    ROS_INFO("Found %lu  plane inliers out of %lu points in the cleaned cloud",
            inliers->indices.size (),
            cloud->size()
        );

    // ############# remove the plane #############
    // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    // http://docs.pointclouds.org/1.9.0/classpcl_1_1_extract_indices.html
    // use setNegative() to remove the plane

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    // remove the inliers
    extract.setNegative (true);
    extract.filter (*cloud);
    ROS_INFO("Processed cloud: width = %d, height = %d\tsize = %lu",
            cloud->width,
            cloud->height,
            cloud->size()
            );

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          objects_color_handler (cloud, 180, 0, 255);
    // show_cloud(cloud, "Objects", objects_color_handler);

    // ############# segment the remaining points #############
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // all the objects
    std::vector<T_PointCloud::Ptr> objects;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
         it != cluster_indices.end ();
         ++it
        ) {
        // create the new cloud for a single object
        T_PointCloud::Ptr cloud_cluster (new T_PointCloud);
        // add all the points
        // MAYBE could you use ExtractIndices ?
        for (std::vector<int>::const_iterator pit = it->indices.begin ();
             pit != it->indices.end ();
             ++pit
            ) {
            cloud_cluster->points.push_back (cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        ROS_INFO("PointCloud representing the Cluster: size = %lu",
                cloud_cluster->points.size());

        objects.push_back(cloud_cluster);
    }
    ROS_INFO("Trovati %lu oggetti",objects.size ());

    // for misterious reasons this complains
    // for (std::vector<T_PointCloud::Ptr>::const_iterator it = objects.begin ();
         // it != objects.end ();
         // ++it
        // ) {
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
              // objects_color_handler (cloud, 180, 0, 130);
        // // show_cloud(*it, "Objects", objects_color_handler);
    // }
    for(std::vector<T_PointCloud::Ptr>::size_type i = 0; i != objects.size(); i++) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
              objects_color_handler (cloud, 180, 0, 130);
        show_cloud(objects[i], "Objects", objects_color_handler);
    }
}

