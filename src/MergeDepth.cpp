// #include <pluginlib/class_list_macros.h>
#include "MergeDepth.hpp"
#include <ros/ros.h>
#include "std_msgs/Header.h"
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv-3.3.1-dev/opencv/cv.hpp>


using namespace sensor_msgs;
using namespace std_msgs;
using namespace message_filters;

namespace ap34 {
    MergeDepth::MergeDepth(ros::NodeHandle& nh) :
            it_(nh),
            top_image_sub_(it_, "input_top_image", 1),
            bottom_image_sub_(it_, "input_bottom_image", 1),
            img_synchronizer(ImageSyncPolicy(3), top_image_sub_, bottom_image_sub_)
    {
        this->nh = nh;

        img_synchronizer.registerCallback(boost::bind(&MergeDepth::IncomingIMG, this, _1, _2));
        img_pub = nh.advertise<sensor_msgs::Image>("output_merged_image", 1);

        usleep(500000);
        ROS_INFO("Initialized");
    }

    void MergeDepth::IncomingIMG(const sensor_msgs::Image::ConstPtr& img_top, const sensor_msgs::Image::ConstPtr& img_bottom) {
        cv_bridge::CvImagePtr cv_top_ptr;
        cv_bridge::CvImagePtr cv_btm_ptr;

        cv_top_ptr = cv_bridge::toCvCopy(img_top);
        cv_btm_ptr = cv_bridge::toCvCopy(img_bottom);
        if(cv_top_ptr->image.cols != cv_btm_ptr->image.cols) {
            cv_top_ptr->image.release();
            cv_btm_ptr->image.release();
            return;
        }


        auto cv_top_image = cv_top_ptr->image;
        auto cv_btm_image = cv_btm_ptr->image;

        cv::Mat top_nan = (cv_top_image != cv_top_image);
        cv::Mat btm_nan = (cv_btm_image != cv_btm_image);

        cv_top_image.setTo(cv::Scalar(0.), top_nan);
        cv_btm_image.setTo(cv::Scalar(0.), btm_nan);

        cv::Mat top_mask(cv_top_image.size(), 0, cv::Scalar(0));
        cv::Mat btm_mask(cv_btm_image.size(), 0, cv::Scalar(0));

        cv::threshold(cv_top_image, top_mask, 0.0, 255, 0);
        cv::threshold(cv_btm_image, btm_mask, 0.0, 255, 0);

        top_mask.convertTo(top_mask, CV_8U);
        btm_mask.convertTo(btm_mask, CV_8U);

        cv::Mat target(cv_top_image.rows + cv_btm_image.rows, cv_top_image.cols, cv_top_image.type());
        cv_top_image.copyTo(target, top_mask);
        cv_btm_image.copyTo(target, btm_mask);

        target = target * 1000.;

        cv::dilate(target, target, cv::Mat(), cv::Point(0, 0), 2, 1, 1);
        cv::erode(target, target, cv::Mat(), cv::Point(0, 0), 2, 1, 1);

        std_msgs::Header header;
        header.frame_id = "dual_xtion_virtual_cam_link";
        header.stamp = cv_top_ptr->header.stamp;
        header.seq = cv_top_ptr->header.seq;

        sensor_msgs::Image msg;
        auto bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, target);
        bridge.toImageMsg(msg);
        img_pub.publish(msg);

        target.release();
        cv_top_image.release();
        cv_btm_image.release();
        top_nan.release();
        btm_nan.release();
        top_mask.release();
        btm_mask.release();
    }
}
