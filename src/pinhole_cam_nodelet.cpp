#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "PinholePCLCam.hpp"


#undef ROS_DEBUG
#undef ROS_INFO
#undef ROS_ERROR
#undef ROS_WARN

#define ROS_DEBUG NODELET_DEBUG
#define ROS_INFO NODELET_INFO
#define ROS_ERROR NODELET_ERROR
#define ROS_WARN NODELET_WARN

namespace ap34 {
  class PinholePCLCamNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit() {
        NODELET_INFO("Initializing PinholePCLCamNodelet nodelet...");
        nh = getPrivateNodeHandle();
        this->pinholeCam = new PinholePCLCam(nh);
    }

    PinholePCLCamNodelet() : Nodelet() {}

  private:
    ros::NodeHandle nh;
    PinholePCLCam* pinholeCam;
  };

}
PLUGINLIB_EXPORT_CLASS(ap34::PinholePCLCamNodelet, nodelet::Nodelet);
