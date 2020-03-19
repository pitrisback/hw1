#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

// #include <pcl/impl/point_types.hpp>

// tutorials
// http://wiki.ros.org/pcl_ros
// http://wiki.ros.org/pcl/Tutorials

typedef pcl::PointXYZRGB T_Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> T_PointCloud;

std::vector<int> forme_interessanti;

void show_cloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                T_PointCloud::Ptr & cloud,
                std::string cloud_name,
                pcl::visualization::PointCloudColorHandlerCustom<T_Point> cloud_color_handler,
                bool spin_now=true
        ) {
    // std::string viewer_name = "3D Viewer";
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));
  
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<T_Point> (cloud, cloud_color_handler, cloud_name);

    viewer->spinOnce ( 1 );
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

    // ROS_INFO("Filtering along %s in range (%f, %f)", axis.c_str(), lower_bound, upper_bound);

    pcl::PassThrough<T_Point> pass_through;
    pass_through.setInputCloud (cloud);
    pass_through.setFilterLimits (lower_bound, upper_bound);
    pass_through.setFilterFieldName (axis);
    pass_through.filter( *cloud );
}

void find_plane(T_PointCloud::Ptr input_cloud,
                pcl::ModelCoefficients::Ptr coefficients,
                pcl::PointIndices::Ptr inliers,
                float distance_threshold=0.01) {
    // http://pointclouds.org/documentation/tutorials/planar_segmentation.php
    // Optional
    // seg.setOptimizeCoefficients (true);
    // Create the segmentation object
    pcl::SACSegmentation<T_Point> seg;

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);

    seg.setInputCloud (input_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    // ROS_INFO("Found %lu  plane inliers out of %lu points in the cleaned cloud",
            // inliers->indices.size (),
            // input_cloud->size()
        // );
}

void remove_plane(T_PointCloud::Ptr input_cloud,
                  T_PointCloud::Ptr remaining_cloud,
                  T_PointCloud::Ptr plane_cloud,
                  pcl::PointIndices::Ptr inliers) {
    // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    // http://docs.pointclouds.org/1.9.0/classpcl_1_1_extract_indices.html
    // use setNegative() to remove the plane

    // ROS_INFO("Input cloud: width = %d, height = %d\tsize = %lu",
            // input_cloud->width,
            // input_cloud->height,
            // input_cloud->size()
            // );
    // ROS_INFO("Inliers size %lu", inliers->indices.size());

    // Create the filtering object
    pcl::ExtractIndices<T_Point> extractor;
    extractor.setInputCloud (input_cloud);
    extractor.setIndices (inliers);

    // remove the inliers
    extractor.setNegative (true);
    extractor.filter (*remaining_cloud);
    // ROS_INFO("Remaining cloud: width = %d, height = %d\tsize = %lu",
            // remaining_cloud->width,
            // remaining_cloud->height,
            // remaining_cloud->size()
            // );

    // keep the inliers (plane)
    extractor.setNegative (false);
    extractor.filter (*plane_cloud);
    // ROS_INFO("Plane cloud: width = %d, height = %d\tsize = %lu",
            // plane_cloud->width,
            // plane_cloud->height,
            // plane_cloud->size()
            // );
}

std::vector<T_PointCloud::Ptr> segment_cloud(T_PointCloud::Ptr cloud, float & table_z_coeff) {
    // cloud viewer
    // std::string viewer_name = "3D Viewer - segment cloud";
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> white_color (cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> pla1 (cloud, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> pla2 (cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> pla3 (cloud, 255, 0, 0);

    // ############# chop the cloud #############

    // only keep the points near the table
    filter_cloud(cloud, "z", 1, 2.0);
    filter_cloud(cloud, "y", -0.5, 0.3);
    filter_cloud(cloud, "x", -0.5, 0.5);
    // show_cloud(viewer, cloud, "cloud_chopped", white_color, false);

    // ############# find the plane #############

    float distance_threshold = 0.01;
    pcl::ModelCoefficients::Ptr coefficients_0 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_0 (new pcl::PointIndices);
    find_plane(cloud, coefficients_0, inliers_0, distance_threshold);
    // ROS_INFO("Coeff 0: %f %f %f %f",
            // coefficients_0->values[0],
            // coefficients_0->values[1],
            // coefficients_0->values[2],
            // coefficients_0->values[3]);
    table_z_coeff = coefficients_0->values[3];

    // ############# remove the plane #############

    T_PointCloud::Ptr plane_cloud_0 (new T_PointCloud);
    T_PointCloud::Ptr remaining_cloud_0 (new T_PointCloud);
    remove_plane(cloud, remaining_cloud_0, plane_cloud_0, inliers_0);
    // show_cloud(viewer, remaining_cloud_0, "remaining_cloud_0", pla1, false);

    // ############# segment the remaining points #############
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<T_Point>::Ptr tree (new pcl::search::KdTree<T_Point>);
    tree->setInputCloud (remaining_cloud_0);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T_Point> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (remaining_cloud_0);
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
            // cloud_cluster->points.push_back (cloud->points[*pit]); // remaining_cloud_0 ???
            cloud_cluster->points.push_back (remaining_cloud_0->points[*pit]); // remaining_cloud_0 ???
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // ROS_INFO("PointCloud representing the Cluster: size = %lu", cloud_cluster->points.size());

        objects.push_back(cloud_cluster);
    }
    ROS_INFO("Trovati %lu oggetti",objects.size ());

    return objects;
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

    diedro_angle = diedro_angle * 180 / M_PI;

    return diedro_angle;
}

T_Point compute_mean(T_PointCloud::Ptr some_cloud) {
    float mean_x = 0;
    float mean_y = 0;
    float mean_z = 0;
    T_Point mean;
    mean.x = mean_x;
    mean.y = mean_y;
    mean.z = mean_z;

    if (some_cloud->size() == 0) {
        return mean;
    }

    for (int i=0; i < some_cloud->size(); i++) {
        mean_x += some_cloud->points[i].x;
        mean_y += some_cloud->points[i].y;
        mean_z += some_cloud->points[i].z;
    }
    mean_x /= some_cloud->size();
    mean_y /= some_cloud->size();
    mean_z /= some_cloud->size();

    mean.x = mean_x;
    mean.y = mean_y;
    mean.z = mean_z;
    return mean;
}

float distance_3D(T_Point a, T_Point b) {
    return std::sqrt(
                pow((a.x-b.x), 2) +
                pow((a.y-b.y), 2) +
                pow((a.z-b.z), 2)
            );
}

bool ranger(float x, float value, float epsilon = 2) {
    return std::abs( x - value ) < epsilon;
}

bool abs_ranger(float x, float value, float epsilon = 2) {
    // ritorna true per -0.7 e 0.7 comparati a 0.7
    return std::abs( std::abs(x) - value ) < epsilon;
}

bool trova_vertice(T_PointCloud::Ptr faccia_orizzontale,
                   T_Point centro,
                   float raggio_nuvola,
                   std::vector<T_Point> & vertici_trovati,
                   float epsilon_interno,
                   float epsilon_vertice
                   ) {

    float max_dist = 0;
    int indice_vertice = -1;
    bool trovato = false;
    // ROS_INFO("Inizia ricerca");

    for (int i = 0; i < faccia_orizzontale->size(); i++) {
        float dist_centro = distance_3D(centro, faccia_orizzontale->points[i]);

        if (dist_centro < raggio_nuvola * epsilon_interno ) {
            // punto troppo vicino al centro
            continue;
        }

        // controlla che non sia troppo vicino ai vertici trovati
        bool reject = false;
        for (int j = 0; j < vertici_trovati.size() && !reject; j++) {
            float dist_vert = distance_3D(vertici_trovati[j], faccia_orizzontale->points[i]);
            // ROS_INFO("controllo vertice %d sul punto %d dist_vert %f dist_centro %f", j, i, dist_vert, dist_centro);
            if (dist_vert < raggio_nuvola * epsilon_vertice ) {
                // ROS_INFO("Reject da vertice %d sul punto %d %f", j, i, dist_vert);
                // continue;
                reject = true;
            }
        }
        // if the point is too close to one of the old vertex found, go to the next point
        if (reject) continue;

        // se il punto e' il piu' lontano per ora
        if (dist_centro > max_dist) {
            max_dist = dist_centro;
            indice_vertice = i;
            trovato = true;
            // ROS_INFO("trovato nuovo max_dist temp punto[%d] %f", i, dist_centro);
        }
    }

    if (trovato) {
        // aggiungi il punto valido a massima distanza alla lista di vertici
        vertici_trovati.push_back(faccia_orizzontale->points[indice_vertice]);
        // ROS_INFO("Size vertici_trovati %lu", vertici_trovati.size());
        // ROS_INFO("Aggiunto a vertici_trovat indice %d", indice_vertice);
    }

    return trovato;
}

int conta_vertici(T_PointCloud::Ptr faccia_orizzontale) {
    // ROS_INFO("Inizia conta_vertici");
    // ROS_INFO("Size faccia_orizzontale %lu", faccia_orizzontale->size());

    // cloud viewer
    // std::string viewer_name = "3D Viewer";
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));
    // colors for the clouds
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> c_B (faccia_orizzontale, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> c_G (faccia_orizzontale, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> c_R (faccia_orizzontale, 255, 0, 0);

    // centro della nuvola
    T_Point centro = compute_mean(faccia_orizzontale);

    // trova il raggio
    float raggio_nuvola = 0;
    for (int i = 0; i < faccia_orizzontale->size(); i++) {
        float dist = distance_3D(centro, faccia_orizzontale->points[i]);
        // ROS_INFO("dist centro punto[%d] %f", i, dist);
        if (dist > raggio_nuvola) raggio_nuvola = dist;
    }
    // ROS_INFO("Raggio nuvola %f", raggio_nuvola);

    float epsilon_interno = 0.85;
    float epsilon_vertice = 0.5;

    std::vector<T_Point> vertici_trovati;

    bool trovato = true;
    while (vertici_trovati.size() < 6 && trovato) {
        trovato = trova_vertice(faccia_orizzontale, centro, raggio_nuvola, vertici_trovati, epsilon_interno, epsilon_vertice);
        // ROS_INFO("Size vertici_trovati %lu", vertici_trovati.size());
        // sleep(1);
    }

    // ROS_INFO("Trovati %lu", vertici_trovati.size());
    return vertici_trovati.size();
}

int analyze_color(T_PointCloud::Ptr object) {
    int freq_r = 0;
    int freq_g = 0;
    int freq_b = 0;
    int freq_c = 0;
    int freq_m = 0;
    int freq_y = 0;
    int freq_black = 0;
    int freq_white = 0;
    uint8_t delta = 50;

    for (int i = 0; i < object->size(); i++) {
        uint8_t r = object->points[i].r;
        uint8_t g = object->points[i].g;
        uint8_t b = object->points[i].b;
        // ROS_INFO("RGB %hhu %hhu %hhu", r, g, b);

        if (r > g + delta && r > b + delta) {
            // red molto rossi
            freq_r += 1;
        } else if (g > r + delta && g > b + delta) {
            // green molto verdi
            freq_g += 1;
        } else if (b > r + delta && b > g + delta) {
            // blue molto blu
            freq_b += 1;
        } else if (g > r + delta && b > r + delta) {
            // ciano = green + blue
            freq_c += 1;
        } else if (g > b + delta && r > b + delta) {
            // yellow = green + red
            freq_y += 1;
        } else if (r > g + delta && b > g + delta) {
            // magenta = red + blue
            freq_m += 1;
        } else if (r < delta && g < delta && b < delta ) {
            // black
            freq_black += 1;
        } else {
            // white lol
            freq_white += 1;
        }
    }
    // ROS_INFO("Freq RGB %d %d %d CMY %d %d %d BW %d %d", freq_r, freq_g, freq_b, freq_c, freq_m, freq_y, freq_black, freq_white);

    int freq [6] = { freq_r, freq_g, freq_b, freq_c, freq_m, freq_y };
    int max_index = 0;
    for (int i = 0; i < 6; i++) {
        if (freq[i] > freq[max_index]) {
            max_index = i;
        }
    }
    // ROS_INFO("Max index %d", max_index);
    return max_index;
}

int analyze_object(T_PointCloud::Ptr object, T_Point & the_pose, float table_z_coeff) {
    // colors for the clouds
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> pla1 (object, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> pla2 (object, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> pla3 (object, 255, 0, 0);

    // cloud viewer
    // std::string viewer_name = "3D Viewer";
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));

    // ROS_INFO("\nAnalyze a %lu points cloud", object->size());

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
    if (remaining_cloud_1->size() > 30) {
        done_2 = true;
        find_plane(remaining_cloud_1, coefficients_2, inliers_2, distance_threshold);
        remove_plane(remaining_cloud_1, remaining_cloud_2, plane_cloud_2, inliers_2);

        if (remaining_cloud_2->size() > 30) {
            done_3 = true;
            find_plane(remaining_cloud_2, coefficients_3, inliers_3, distance_threshold);
            remove_plane(remaining_cloud_2, remaining_cloud_3, plane_cloud_3, inliers_3);
        }
    }

    // show_cloud(viewer, remaining_cloud_2, "Remaining_2", rem1, false);
    // show_cloud(viewer, plane_cloud_1, "Plane_1", pla1, false);
    // show_cloud(viewer, plane_cloud_2, "Plane_2", pla2, false);
    // show_cloud(viewer, plane_cloud_3, "Plane_3", pla3, false);

    float diedro12 = 180, diedro13 = 180, diedro23 = 180;

    float c1x = coefficients_1->values[0], c1y = coefficients_1->values[1], c1z = coefficients_1->values[2];
    // ROS_INFO("Coeff 1: %f %f %f", c1x, c1y, c1z);
    float c2x, c2y, c2z;
    float c3x, c3y, c3z;
    if (done_2) {
        float c2x = coefficients_2->values[0], c2y = coefficients_2->values[1], c2z = coefficients_2->values[2];
        // ROS_INFO("Coeff 2: %f %f %f", c2x, c2y, c2z);

        diedro12 = diedro(coefficients_1, coefficients_2);
        // ROS_INFO("Angolo 1 2 %f", diedro12);

        if (done_3) {
            float c3x = coefficients_3->values[0], c3y = coefficients_3->values[1], c3z = coefficients_3->values[2];
            // ROS_INFO("Coeff 3: %f %f %f", c3x, c3y, c3z);
            diedro23 = diedro(coefficients_2, coefficients_3);
            // ROS_INFO("Angolo 2 3 %f", diedro23);
            diedro13 = diedro(coefficients_1, coefficients_3);
            // ROS_INFO("Angolo 1 3 %f", diedro13);
        }
    }

    // parse the angles found
    float sort_diedro_1 = diedro12, sort_diedro_2 = diedro13, sort_diedro_3 = diedro23;
    if (ranger(sort_diedro_1, 120)) sort_diedro_1 -= 60;
    if (ranger(sort_diedro_2, 120)) sort_diedro_2 -= 60;
    if (ranger(sort_diedro_3, 120)) sort_diedro_3 -= 60;
    if (ranger(sort_diedro_1, 135)) sort_diedro_1 -= 90;
    if (ranger(sort_diedro_2, 135)) sort_diedro_2 -= 90;
    if (ranger(sort_diedro_3, 135)) sort_diedro_3 -= 90;
    // bubble sort them
    if (sort_diedro_1 > sort_diedro_2) std::swap(sort_diedro_1, sort_diedro_2);
    if (sort_diedro_2 > sort_diedro_3) std::swap(sort_diedro_2, sort_diedro_3);
    if (sort_diedro_1 > sort_diedro_2) std::swap(sort_diedro_1, sort_diedro_2);
    // ROS_INFO("Sort Angolo 1 %f", sort_diedro_1);
    // ROS_INFO("Sort Angolo 2 %f", sort_diedro_2);
    // ROS_INFO("Sort Angolo 3 %f", sort_diedro_3);

    // 45 90 90 vertice dello spigolo acuto prisma triangolare
    // 60 60 60 prisma esagonale di cui vedo le tre facce laterali
    // 60 90 90 prisma esagonale di cui vedo due laterali e sopra
    // 90 90 90 cubo o spigolo sfighez del prisma triangolare

    float sqrt22 = std::sqrt(2) / 2;
    float sqrt32 = std::sqrt(3) / 2;
    float coeff_orizzontale = 1;

    int type_forma = 0;
    // 1 cubo
    // 2 triangolo
    // 3 esagono

    if (done_2 == false) {
        // only one face found
        // capire il tipo di faccia per distinguere triangolo quadrato esagono
        // triangolo visto molto di lato, esagono in piedi o cubo
        if (abs_ranger(c1z, sqrt22, 0.1)) {
            // triangolo!
            type_forma = 2;
            // ROS_INFO("Triangolo vedo una faccia");
        } else {
            // trova la faccia orizzontale
            T_PointCloud::Ptr faccia_orizzontale;
            if (abs_ranger(c1z, coeff_orizzontale, 0.1)) {
                faccia_orizzontale = plane_cloud_1;
                // ROS_INFO("Prima faccia orizzontale");
            } else if (abs_ranger(c2z, coeff_orizzontale, 0.1)) {
                faccia_orizzontale = plane_cloud_2;
                // ROS_INFO("Seconda faccia orizzontale");
            } else {
                faccia_orizzontale = plane_cloud_3;
                // ROS_INFO("Terzo faccia orizzontale");
            }
            // conta i vertici
            int vertici = conta_vertici(faccia_orizzontale);
            if (vertici == 4) {
                // cubo!
                type_forma = 1;
                // ROS_INFO("Cubo vedo una faccia");
            } else {
                // esagono
                type_forma = 3;
                // ROS_INFO("Esagono verticale vedo una faccia");
            }
        }
    } else {
        if (done_3 == false) {
            // two faces found
            // puo' essere triangolo, cubo o esagono in piedi, o disteso
            if (abs_ranger(c1z, sqrt22, 0.1) || abs_ranger(c2z, sqrt22, 0.1)) {
                // triangolo!
                type_forma = 2;
                // ROS_INFO("Triangolo vedo due facce");
            } else if (abs_ranger(c1z, sqrt32, 0.1) || abs_ranger(c2z, sqrt32, 0.1)) {
                // esagono disteso
                type_forma = 3;
                // ROS_INFO("Esagono disteso vedo due facce");
            } else {
                if (abs_ranger(c1z, coeff_orizzontale, 0.1)) {
                    the_pose = compute_mean(plane_cloud_1);
                } else {
                    the_pose = compute_mean(plane_cloud_2);
                }
                float z_faccia = - (table_z_coeff + the_pose.z);
                // ROS_INFO("vedo due facce coeff z_faccia %f, the_pose.z %f", z_faccia, the_pose.z);
                if (z_faccia > 0.15) {
                    // esagono
                    type_forma = 3;
                    // ROS_INFO("Esagono in piedi vedo due facce");
                } else {
                    // cubo!
                    type_forma = 1;
                    // ROS_INFO("Cubo vedo due facce");
                }
            }
        } else {
            // three faces found
            if (ranger(sort_diedro_1, 45)) {
                // e' un triangolo di sicuro
                type_forma = 2;
                // ROS_INFO("Triangolo vedo tre facce, dal lato fortunato");
            } else if (ranger(sort_diedro_1, 60)) {
                // e' un esagono di sicuro
                type_forma = 3;
                // ROS_INFO("Esagono vedo tre facce");
            } else {
                // forse cubo, forse triangolo
                // analizzo il coefficiente sulla z e vedo se e' vicino a sqrt(2)/2
                if (abs_ranger(c1z, sqrt22, 0.1) || abs_ranger(c2z, sqrt22, 0.1) || abs_ranger(c3z, sqrt22, 0.1)) {
                    // triangolo!
                    type_forma = 2;
                    // ROS_INFO("Triangolo vedo tre facce, dal lato sfortunato");
                } else {
                    // cubo!
                    type_forma = 1;
                    // ROS_INFO("Cubo vedo tre facce");
                }
            }
        }
    }

    // estrae la pose dell'oggetto
    if (done_2 == false) {
        // ha trovato solo una faccia, la pose e' data dalla media della cloud
        the_pose = compute_mean(plane_cloud_1);
    } else {
        if (done_3 == false) {
            // two faces found
            if (type_forma == 2) {
                // triangolo
                if (abs_ranger(c1z, sqrt22, 0.1)) {
                    // la prima faccia trovata era a 45 gradi, usa quella come pose
                    the_pose = compute_mean(plane_cloud_1);
                } else {
                    the_pose = compute_mean(plane_cloud_2);
                }
            } else {
                // esagono o cubo: c'e' in entrambi i casi una faccia orizzontale
                if (abs_ranger(c1z, coeff_orizzontale, 0.1)) {
                    // la prima faccia trovata era orizzontale, usa quella come pose
                    the_pose = compute_mean(plane_cloud_1);
                } else {
                    // era la seconda orizzontale
                    the_pose = compute_mean(plane_cloud_2);
                }
            }
        } else {
            // three faces found
            if (type_forma == 2) {
                // triangolo, cerca una faccia a 45 gradi
                if (abs_ranger(c1z, sqrt22, 0.1)) {
                    // la prima faccia trovata era a 45 gradi, usa quella come pose
                    the_pose = compute_mean(plane_cloud_1);
                } else if (abs_ranger(c2z, sqrt22, 0.1)) {
                    the_pose = compute_mean(plane_cloud_2);
                } else {
                    the_pose = compute_mean(plane_cloud_3);
                }
            } else {
                // esagono o cubo: c'e' in entrambi i casi una faccia orizzontale
                if (abs_ranger(c1z, coeff_orizzontale, 0.1)) {
                    // la prima faccia trovata era orizzontale, usa quella come pose
                    the_pose = compute_mean(plane_cloud_1);
                } else if (abs_ranger(c2z, coeff_orizzontale, 0.1)) {
                    the_pose = compute_mean(plane_cloud_2);
                } else {
                    the_pose = compute_mean(plane_cloud_3);
                }
            }
        }
    }

    int color = analyze_color(object);

    int forma_trovata = -1;

    if (type_forma == 1) {
        // cubi
        if (color == 0) {
            // ROS_INFO("Cubo rosso");
            forma_trovata = 0;
        } else if (color == 2) {
            // ROS_INFO("Cubo blue");
            forma_trovata = 3;
        } else {
            // ROS_INFO("Cubo FALLITO colore incompatibile");
        }
    } else if (type_forma == 2) {
        // triangoli
        if (color == 0) {
            // ROS_INFO("Triangolo rosso");
            forma_trovata = 4;
        } else if (color == 1) {
            // ROS_INFO("Triangolo verde");
            forma_trovata = 2;
        } else {
            // ROS_INFO("Triangolo FALLITO colore incompatibile");
        }
    } else if (type_forma == 3) {
        // esagoni
        if (color == 5) {
            // ROS_INFO("Esagono giallo");
            forma_trovata = 1;
        } else {
            // ROS_INFO("Esagono FALLITO colore incompatibile");
        }
    }

    return forma_trovata;

}

void callback_main(const T_PointCloud::ConstPtr& msg) {
    // ROS_INFO("Msg: width = %d, height = %d", msg->width, msg->height);
    // copyPointCloud https://github.com/PointCloudLibrary/pcl/blob/master/examples/common/example_copy_point_cloud.cpp
    T_PointCloud::Ptr cloud (new T_PointCloud);
    copyPointCloud(*msg, *cloud);
    ROS_INFO("\nRicevuta cloud: width = %d, height = %d", cloud->width, cloud->height);

    // cloud viewer
    // std::string viewer_name = "3D Viewer";
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));
    pcl::visualization::PointCloudColorHandlerCustom<T_Point> objects_color_handler (cloud, 180, 0, 130);

    float table_z_coeff;
    std::vector<T_PointCloud::Ptr> objects = segment_cloud(cloud, table_z_coeff);

    // salva quante forme per ogni tipo sono state trovate
    std::map<int, int> forme_trovate;
    forme_trovate.insert(std::pair<int, int> (-1, 0)); // trovato niente
    forme_trovate.insert(std::pair<int, int> (0, 0)); // cubo rosso
    forme_trovate.insert(std::pair<int, int> (1, 0)); // cilindro giallo
    forme_trovate.insert(std::pair<int, int> (2, 0)); // triangolo verde
    forme_trovate.insert(std::pair<int, int> (3, 0)); // cubo blu
    forme_trovate.insert(std::pair<int, int> (4, 0)); // triangolo rosso

    // nomi delle forme
    std::map<int, std::string> ids2names;
    ids2names.insert(std::pair<int, std::string> (-1, "not_identified"));
    ids2names.insert(std::pair<int, std::string> (0, "red_cube"));
    ids2names.insert(std::pair<int, std::string> (1, "yellow_cyl"));
    ids2names.insert(std::pair<int, std::string> (2, "green_prism"));
    ids2names.insert(std::pair<int, std::string> (3, "blue_cube"));
    ids2names.insert(std::pair<int, std::string> (4, "red_prism"));

    for(std::vector<T_PointCloud::Ptr>::size_type i = 0; i != objects.size(); i++) {
        // pose dell'oggetto
        T_Point the_pose;

        ROS_INFO("Analizzo object %lu", i);
        // show_cloud(viewer, objects[i], "Objects", objects_color_handler, false);
        int forma_trovata = analyze_object(objects[i], the_pose, table_z_coeff);
        forme_trovate[forma_trovata]++;

        // correct the pose, relative to the table top
        the_pose.z = - (table_z_coeff + the_pose.z);
        ROS_INFO("Trovata forma %s, pose: %f %f %f", ids2names[forma_trovata].c_str(), the_pose.x, the_pose.y, the_pose.z);
    }

    // sleep(1000);

    ROS_INFO("\nRecap:");
    // scorre tutte le forme da trovare
    for (int i = 0; i < forme_interessanti.size(); i++) {
        // estrae la forma corrente da esaminare per chiarezza
        int forma_corrente = forme_interessanti[i];

        if (forme_trovate[forma_corrente] == 0) {
            // se il numero di trovate e' sceso a 0 (o lo e' sempre stato), non c'e' la forma nella scena
            // ROS_INFO("Non ho trovato %d", forma_corrente);
            ROS_INFO("Non ho trovato %s", ids2names[forma_corrente].c_str());
        } else {
            // la forma e' stata trovata nella scena
            // ROS_INFO("Trovato %d!!!", forma_corrente);
            ROS_INFO("Trovato %s", ids2names[forma_corrente].c_str());
            forme_trovate[forma_corrente]--;
        }
    }
}

int main(int argc, char** argv) {
    ROS_INFO("Booting pcl_find_objects");

    std::map<std::string, int> names2ids;
    names2ids.insert(std::pair<std::string, int> ("red_cube", 0));
    names2ids.insert(std::pair<std::string, int> ("yellow_cyl", 1));
    names2ids.insert(std::pair<std::string, int> ("green_prism", 2));
    names2ids.insert(std::pair<std::string, int> ("blue_cube", 3));
    names2ids.insert(std::pair<std::string, int> ("red_prism", 4));

    for(int i = 1; i<argc; i++){
        if(names2ids.count(argv[i]) == 1) {
            forme_interessanti.push_back(names2ids[argv[i]]);
            ROS_INFO("Adding tag %d for object %s", names2ids[argv[i]], argv[i]);
        } else {
            ROS_INFO("NOT adding tag %d for object %s", names2ids[argv[i]], argv[i]);
        }
    }

    // ros::init(argc, argv, "sub_pcl");
    ros::init(argc, argv, "pcl_find_objects");
    ros::NodeHandle nh;

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
