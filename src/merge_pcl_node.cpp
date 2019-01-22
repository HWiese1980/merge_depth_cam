#include "MergePCL.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_pcl");
    ros::NodeHandle nh("~");
    ap34::MergePCL id(nh);
    ros::spin();
}


