#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "MergePCL.hpp"


#undef ROS_DEBUG
#undef ROS_INFO
#undef ROS_ERROR
#undef ROS_WARN

#define ROS_DEBUG NODELET_DEBUG
#define ROS_INFO NODELET_INFO
#define ROS_ERROR NODELET_ERROR
#define ROS_WARN NODELET_WARN

namespace ap34 {
  class MergePCLNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit() {
        NODELET_INFO("Initializing MergePCLNodelet nodelet...");
        nh = getPrivateNodeHandle();
        this->mergePCL = new MergePCL(nh);
    }

    MergePCLNodelet() : Nodelet() {}

  private:
    ros::NodeHandle nh;
    MergePCL* mergePCL;
  };

}
PLUGINLIB_EXPORT_CLASS(ap34::MergePCLNodelet, nodelet::Nodelet);
