#include "MergeDepth.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_depth");
    ros::NodeHandle nh("~");
    ap34::MergeDepth id(nh);
    ros::spin();
}


