#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <April2Pose.h>
#include "ros/package.h"
#include <sys/stat.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "april_parse_listener");
    ROS_INFO("Setting up");
    
    // map from names to ids
    std::map<std::string, int> names2ids;
    names2ids.insert(std::pair<std::string, int> ("red_cube_0", 0));
    names2ids.insert(std::pair<std::string, int> ("red_cube_1", 1));
    names2ids.insert(std::pair<std::string, int> ("red_cube_2", 2));
    names2ids.insert(std::pair<std::string, int> ("red_cube_3", 3));
    names2ids.insert(std::pair<std::string, int> ("yellow_cyl_0", 4));
    names2ids.insert(std::pair<std::string, int> ("yellow_cyl_1", 5));
    names2ids.insert(std::pair<std::string, int> ("green_prism_0", 6));
    names2ids.insert(std::pair<std::string, int> ("green_prism_1", 7));
    names2ids.insert(std::pair<std::string, int> ("green_prism_2", 8));
    names2ids.insert(std::pair<std::string, int> ("blue_cube_0", 9));
    names2ids.insert(std::pair<std::string, int> ("blue_cube_1", 10));
    names2ids.insert(std::pair<std::string, int> ("blue_cube_2", 11));
    names2ids.insert(std::pair<std::string, int> ("blue_cube_3", 12));
    names2ids.insert(std::pair<std::string, int> ("red_prism_0", 13));
    names2ids.insert(std::pair<std::string, int> ("red_prism_1", 14));
    names2ids.insert(std::pair<std::string, int> ("red_prism_2", 15));

    std::vector<int> searching_ids;
    for(int i = 1; i<argc; i++){
        if(names2ids.count(argv[i]) == 1) {
            searching_ids.push_back(names2ids[argv[i]]);
            ROS_INFO("Adding tag %d for object %s", names2ids[argv[i]], argv[i]);
        } else {
            ROS_INFO("NOT adding tag %d for object %s", names2ids[argv[i]], argv[i]);
        }
    }

    // create the output folder
    std::stringstream ss;
    ss << ros::package::getPath("hw1") << "/output";
    mkdir(ss.str().c_str(), 0777);

    // build the April2Pose object with the list of apriltag ids to look for
    April2Pose apposer = April2Pose(searching_ids);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("tag_detections", 1000, 
        &April2Pose::aprilCallback, &apposer);
    ros::spin();
    return 0;
}
