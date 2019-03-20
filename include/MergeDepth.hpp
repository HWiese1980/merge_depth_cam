#ifndef __MERGE_DEPTH_HPP__
#define __MERGE_DEPTH_HPP__
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImageSyncPolicy;

namespace ap34 {
    class MergeDepth {
    public:
        MergeDepth(ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it_;
        int seq = 0;

        ros::Publisher img_pub;

        typedef image_transport::SubscriberFilter ImageSubscriber;

        ImageSubscriber top_image_sub_;
        ImageSubscriber bottom_image_sub_;

        message_filters::Synchronizer<ImageSyncPolicy> img_synchronizer;

        void IncomingIMG(const sensor_msgs::Image::ConstPtr& image_top, const sensor_msgs::Image::ConstPtr& image_bottom);
    };
}

#endif
