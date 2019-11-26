#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include <April2Pose.h>


April2Pose::April2Pose(int _id){
    ids.push_back(_id);
}

April2Pose::April2Pose(std::vector<int> _ids){
    for(std::vector<int>::const_iterator it = _ids.begin();
        it != _ids.end();
        ++it ) {
            ids.push_back(*it);
        }
}


