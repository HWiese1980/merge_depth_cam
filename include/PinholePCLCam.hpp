
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <dynamic_reconfigure/server.h>
#include <merged_depth_cam/PinholePCLCamConfig.h>

#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"

namespace ap34 {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    void PCL2Mat(const PointCloud::ConstPtr& ptr_cld, cv::Mat& depth_image, int original_width, int original_height, float focal_length);

    class PinholePCLCam {
    public:
        PinholePCLCam(ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh;

        dynamic_reconfigure::Server<merged_depth_cam::PinholePCLCamConfig> server;
        dynamic_reconfigure::Server<merged_depth_cam::PinholePCLCamConfig>::CallbackType cb_type;

        void dynconf_cb(merged_depth_cam::PinholePCLCamConfig &config, uint32_t level);
        void PCL2Mat(const PointCloud::ConstPtr& ptr_cld, cv::Mat& depth_image, int original_width, int original_height);

        float focal_length;
        std::string virtual_cam_frame;

        int stride = 1;
        int target_width = 320;
        float rescale_factor = 1.0F;
        float _upscale_factor = 1.0F;
        float _downscale_factor = 1.0F;
        int blob_size = 1;
        
        int diw, dih;
        int seq = 0;

        image_transport::Publisher pub;

        ros::Subscriber pcl_sub;
        image_transport::Publisher depth_pub;

        void IncomingPCL(const PointCloud::ConstPtr& pcl);
    private: // Camera Data
        cv::Mat P;
        cv::Mat K;
        cv::Mat rvec;
        cv::Mat Thomogeneous;
        cv::Mat T;
        cv::Mat distCoeffs;
        cv::Mat rvecR;

    };
}
