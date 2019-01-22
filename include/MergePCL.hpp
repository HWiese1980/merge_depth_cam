#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> PCLSyncPolicy;


namespace ap34 {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    class MergePCL {
    public:
        MergePCL(ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        float focal_length;
        std::string target_frame;

        int seq = 0;

        ros::Publisher pcl_pub;

        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_a;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_b;

        message_filters::Synchronizer<PCLSyncPolicy> pcl_synchronizer;

        void IncomingPCL(const sensor_msgs::PointCloud2::ConstPtr& pclA, const sensor_msgs::PointCloud2::ConstPtr& pclB);
    };
}
