#include "PinholePCLCam.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pinhole_cam");
    ros::NodeHandle nh("~");
    ap34::PinholePCLCam id(nh);
    ros::spin();
}


