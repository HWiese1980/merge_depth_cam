#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "MergeDepth.hpp"


#undef ROS_DEBUG
#undef ROS_INFO
#undef ROS_ERROR
#undef ROS_WARN

#define ROS_DEBUG NODELET_DEBUG
#define ROS_INFO NODELET_INFO
#define ROS_ERROR NODELET_ERROR
#define ROS_WARN NODELET_WARN

namespace ap34 {
  class MergeDepthNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit() {
        NODELET_INFO("Initializing MergeDepthNodelet nodelet...");
        nh = getPrivateNodeHandle();
        this->mergeDepth = new MergeDepth(nh);
    }

    MergeDepthNodelet() : Nodelet() {}

  private:
    ros::NodeHandle nh;
    MergeDepth* mergeDepth;
  };

}
PLUGINLIB_EXPORT_CLASS(ap34::MergeDepthNodelet, nodelet::Nodelet);
