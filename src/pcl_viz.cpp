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

void show_cloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                T_PointCloud::Ptr & cloud,
                std::string cloud_name,
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler,
                bool spin_now=true
        ) {
    // std::string viewer_name = "3D Viewer";
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));
  
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, cloud_name);

    if (spin_now) {
        viewer->initCameraParameters ();
        viewer->addCoordinateSystem (1.0);
        while (!viewer->wasStopped ())
            viewer->spinOnce ( 1 );
    }
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

void find_plane(T_PointCloud::Ptr input_cloud,
                pcl::ModelCoefficients::Ptr coefficients,
                pcl::PointIndices::Ptr inliers,
                float distance_threshold=0.01) {
    // Optional
    // seg.setOptimizeCoefficients (true);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);

    seg.setInputCloud (input_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    ROS_INFO("Found %lu  plane inliers out of %lu points in the cleaned cloud",
            inliers->indices.size (),
            input_cloud->size()
        );
}

void remove_plane(T_PointCloud::Ptr input_cloud,
                  T_PointCloud::Ptr remaining_cloud,
                  T_PointCloud::Ptr plane_cloud,
                  pcl::PointIndices::Ptr inliers) {

    ROS_INFO("Input cloud: width = %d, height = %d\tsize = %lu",
            input_cloud->width,
            input_cloud->height,
            input_cloud->size()
            );
    ROS_INFO("Inliers size %lu", inliers->indices.size());

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud (input_cloud);
    extractor.setIndices (inliers);

    // remove the inliers
    extractor.setNegative (true);
    extractor.filter (*remaining_cloud);
    ROS_INFO("Remaining cloud: width = %d, height = %d\tsize = %lu",
            remaining_cloud->width,
            remaining_cloud->height,
            remaining_cloud->size()
            );

    // keep the inliers (plane)
    extractor.setNegative (false);
    extractor.filter (*plane_cloud);
    ROS_INFO("Plane cloud: width = %d, height = %d\tsize = %lu",
            plane_cloud->width,
            plane_cloud->height,
            plane_cloud->size()
            );
}

float diedro(pcl::ModelCoefficients::Ptr c1,
             pcl::ModelCoefficients::Ptr c2
        ) {
    // the DIEDRO angle is 
    // n2' = -n2         ~ dot product
    // cos(n1 n2') = (n1 . n2')/(|n1||n2'|)
    std::vector<float> n1, n2;
    n1.push_back(c1->values[0]);
    n1.push_back(c1->values[1]);
    n1.push_back(c1->values[2]);
    n2.push_back(c2->values[0]);
    n2.push_back(c2->values[1]);
    n2.push_back(c2->values[2]);

    // ROS_INFO("n1 %f", n1[0]);

    float n1n2 = std::inner_product(n1.begin(), n1.end(), n2.begin(), 0.0);
    // ROS_INFO("n1n2 %f", n1n2);

    float norm1 = std::sqrt(std::inner_product(n1.begin(), n1.end(), n1.begin(), 0.0));
    float norm2 = std::sqrt(std::inner_product(n2.begin(), n2.end(), n2.begin(), 0.0));
    // ROS_INFO("norm1 %f norm2 %f", norm1, norm2);

    float diedro_angle = acos(n1n2 / ( norm1 * norm2));
    // ROS_INFO("diedro_angle %f", diedro_angle * 180 / 3.1415926535);

    return diedro_angle;
}


void analyze_object(T_PointCloud::Ptr object) {
    // colors for the clouds
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pla1 (object, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pla2 (object, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pla3 (object, 255, 0, 0);

    // cloud viewer
    std::string viewer_name = "3D Viewer";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));

    ROS_INFO("\nAnalyze a %lu points cloud", object->size());

    // distance_threshold for RANSAC
    float distance_threshold = 0.001;

    // find the first plane
    pcl::ModelCoefficients::Ptr coefficients_1 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_1 (new pcl::PointIndices);
    find_plane(object, coefficients_1, inliers_1, distance_threshold);

    // split the cloud in first plane and remaining
    T_PointCloud::Ptr plane_cloud_1 (new T_PointCloud);
    T_PointCloud::Ptr remaining_cloud_1 (new T_PointCloud);
    remove_plane(object, remaining_cloud_1, plane_cloud_1, inliers_1);

    // setup second and third planes
    pcl::ModelCoefficients::Ptr coefficients_2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices);
    T_PointCloud::Ptr plane_cloud_2 (new T_PointCloud);
    T_PointCloud::Ptr remaining_cloud_2 (new T_PointCloud);
    pcl::ModelCoefficients::Ptr coefficients_3 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_3 (new pcl::PointIndices);
    T_PointCloud::Ptr plane_cloud_3 (new T_PointCloud);
    T_PointCloud::Ptr remaining_cloud_3 (new T_PointCloud);

    bool done_2 = false;
    bool done_3 = false;

    // if the remaining_cloud_1 is there, find the second plane
    if (remaining_cloud_1->size() > 0) {
        done_2 = true;
        find_plane(remaining_cloud_1, coefficients_2, inliers_2, distance_threshold);
        remove_plane(remaining_cloud_1, remaining_cloud_2, plane_cloud_2, inliers_2);

        if (remaining_cloud_2->size() > 0) {
            done_3 = true;
            find_plane(remaining_cloud_2, coefficients_3, inliers_3, distance_threshold);
            remove_plane(remaining_cloud_2, remaining_cloud_3, plane_cloud_3, inliers_3);
        }
    }

    // show_cloud(viewer, remaining_cloud_2, "Remaining_2", rem1, false);
    show_cloud(viewer, plane_cloud_1, "Plane_1", pla1, false);
    show_cloud(viewer, plane_cloud_2, "Plane_2", pla2, false);
    show_cloud(viewer, plane_cloud_3, "Plane_3", pla3);

    float diedro12, diedro13, diedro23;
    ROS_INFO("Coeff 1: %f %f %f", coefficients_1->values[0], coefficients_1->values[1], coefficients_1->values[2]);
    if (done_2) {
        ROS_INFO("Coeff 2: %f %f %f", coefficients_2->values[0], coefficients_2->values[1], coefficients_2->values[2]);

        diedro12 = diedro(coefficients_1, coefficients_2);
        ROS_INFO("Angolo 1 2 %f", diedro12 * 180 / M_PI);

        if (done_3) {
            ROS_INFO("Coeff 3: %f %f %f", coefficients_3->values[0], coefficients_3->values[1], coefficients_3->values[2]);
            diedro23 = diedro(coefficients_2, coefficients_3);
            ROS_INFO("Angolo 2 3 %f", diedro23 * 180 / M_PI);
            diedro13 = diedro(coefficients_1, coefficients_3);
            ROS_INFO("Angolo 1 3 %f", diedro13 * 180 / M_PI);
        }
    }
}

int main(int argc, char** argv) {
    ROS_INFO("Booting pcl_viz");

    // ############# load the cloud #############

    std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_kinect.pcd";
    // std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_kinect_3obj.pcd";
    // std::string pcl_data_file = "/home/ros/ros_ws/src/hw1/pcl_data/pcl_test_from_mesh.pcd";

    T_PointCloud::Ptr cloud(new T_PointCloud);
    pcl::io::loadPCDFile(pcl_data_file, *cloud);

    ROS_INFO("Loaded cloud: width = %d, height = %d", cloud->width, cloud->height);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          orig_cloud_color_handler (cloud, 255, 255, 255);
    // show_cloud(cloud, "Original", orig_cloud_color_handler);

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
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
              // objects_color_handler (cloud, 180, 0, 130);
        // show_cloud(objects[i], "Objects", objects_color_handler);
        analyze_object(objects[i]);
    }
}

