// #include <pluginlib/class_list_macros.h>
#include "PinholePCLCam.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image_planar.h>
#include <cassert>
#include <cmath>

using namespace sensor_msgs;
using namespace std_msgs;

namespace ap34 {
    void store_image(std::string fname, cv::Mat& img) {
        ROS_INFO_STREAM("Storing image");
        std::stringstream ss_yml, ss_png;
        ss_yml << fname << ".yml";
        ss_png << fname << ".png";

        cv::FileStorage file(ss_yml.str().c_str(), cv::FileStorage::WRITE);
        file << "depth_image" << img;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        cv::Mat write_image;
        cv::normalize(img, write_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::imwrite(ss_png.str().c_str(), write_image, compression_params);
    }

    void PinholePCLCam::PCL2Mat(const PointCloud::ConstPtr& ptr_cld, cv::Mat& depth_image, int original_width, int original_height)
    {
        int blob_offset_u = (blob_size - (blob_size%2)) / 2;
        int blob_offset_v = (blob_size - (blob_size%2)) / 2;

        std::vector<cv::Point2f> projectedPoints;
        std::vector<cv::Point3f> cv_points;
        for(const auto& point : ptr_cld->points) {
            cv_points.push_back(cv::Point3f(point.x, point.y, point.z));
        }

        cv::projectPoints(cv_points, rvecR, T, K, distCoeffs, projectedPoints);
        cv::Rect valid_range(blob_offset_u, blob_offset_v, original_width - (2*blob_offset_u), original_height - (2*blob_offset_v));
        int mid_x = original_width/2;
        int mid_y = original_height/2;

        #pragma omp parallel for
        for(int i = 0; i < ptr_cld->points.size(); i += stride)
        {
            auto point2d = projectedPoints[i];

            int u = (int)(point2d.x + mid_x);
            int v = (int)(point2d.y + mid_y);

            cv::Point p(u, v);
            if(!valid_range.contains(p)) continue;

            float z = ptr_cld->points[i].z;
            float new_val = (float)(z * 1000.0);
            float current_val = depth_image.at<float>(v, u);
            if(current_val < std::numeric_limits<float>::epsilon() || new_val < current_val)
                // only set depth value if current pixel is unset or the new value is closer to the cam (z-buffer)
                // depth_image(cv::Rect(u-blob_offset_u,v-blob_offset_v,2*blob_offset_u,2*blob_offset_v)).setTo(new_val);
                cv::circle(depth_image, p, blob_size/2, new_val, -1);
        }
    }

    void PinholePCLCam::dynconf_cb(merged_depth_cam::PinholePCLCamConfig &config, uint32_t level) {
        ROS_INFO("[PinholePCLCam] Reconfiguring");
        ROS_INFO("  Stride         : %d", config.stride);
        ROS_INFO("  Blob Size      : %d", config.blob_size);
        ROS_INFO("  Focal Length   : %3.3f", config.focal_length);
        ROS_INFO("  Rescale Factor : %3.3f", config.rescale_factor);
        stride = config.stride;
        blob_size = config.blob_size;
        focal_length = config.focal_length;
        rescale_factor = config.rescale_factor;

        float _upscale_factor_internal = (float)target_width / diw;
        ROS_INFO("[Pinhole Cam] Internal upscale factor : %3.3f", _upscale_factor_internal);
        _downscale_factor = rescale_factor;
        _upscale_factor = _upscale_factor_internal / _downscale_factor;

        P.at<float>(0,0) = focal_length / 2;
        P.at<float>(1,1) = focal_length / 2;
        P.at<float>(2,2) = 1;

        cv::decomposeProjectionMatrix(P, K, rvec, Thomogeneous);
        cv::Rodrigues(rvec, rvecR);
    }

    PinholePCLCam::PinholePCLCam(ros::NodeHandle& nh) {
        this->nh = nh;
        image_transport::ImageTransport it(this->nh);

        pcl_sub = nh.subscribe("input_pcl", 1, &PinholePCLCam::IncomingPCL, this);
        depth_pub = it.advertise("output_merged_depth", 1);

        P = cv::Mat(3, 4, cv::DataType<float>::type, cv::Scalar(0.));
        K = cv::Mat(3, 3, cv::DataType<float>::type, cv::Scalar(0.));
        rvec = cv::Mat(3, 3, cv::DataType<float>::type, cv::Scalar(0.));
        Thomogeneous = cv::Mat(4, 1, cv::DataType<float>::type, cv::Scalar(0.));
        T = cv::Mat(3, 1, cv::DataType<float>::type, cv::Scalar(0.));
        distCoeffs = cv::Mat(4, 1, cv::DataType<float>::type, cv::Scalar(0.));
        rvecR = cv::Mat(3, 1, cv::DataType<float>::type, cv::Scalar(0.));

        usleep(500000);

        nh.param<std::string>("virtual_cam_frame", virtual_cam_frame, "virtual_cam_link");
        nh.param<int>("depth_image_width", diw, 640);
        nh.param<int>("depth_image_height", dih, 960);
        nh.param<int>("stride", stride, 1);
        nh.param<int>("target_width", target_width, 320);
        nh.param<int>("blob_size", blob_size, 1);
        nh.param<float>("focal_length", focal_length, 1000.0);
        nh.param<float>("rescale_factor", rescale_factor, 1.0);

        float _upscale_factor_internal = (float)target_width / diw;
        ROS_INFO("[Pinhole Cam] Internal upscale factor : %3.3f", _upscale_factor_internal);
        _downscale_factor = rescale_factor;
        _upscale_factor = _upscale_factor_internal / _downscale_factor;

        ROS_INFO("[Pinhole Cam] Downscale Factor        : %3.3f", _downscale_factor);
        ROS_INFO("[Pinhole Cam] Upscale Factor          : %3.3f", _upscale_factor);
        ROS_INFO("[Pinhole Cam] Blob Size               : %d", blob_size);

        cb_type = boost::bind(&PinholePCLCam::dynconf_cb, this, _1, _2);
        server.setCallback(cb_type);

    }

    void PinholePCLCam::IncomingPCL(const PointCloud::ConstPtr& pcl) {
        if(diw <= 0 || dih <= 0) return;

        cv::Mat depth_image(dih, diw, CV_32FC1, cv::Scalar(0.0));
        PCL2Mat(pcl, depth_image, diw, dih);

        cv::Mat resized_image;

        if(fabs(1.0F - _upscale_factor) >= std::numeric_limits<float>::epsilon())
        {
            cv::resize(depth_image, resized_image, cv::Size(), _downscale_factor, _downscale_factor, CV_INTER_AREA);
            cv::resize(resized_image, resized_image, cv::Size(), _upscale_factor, _upscale_factor, CV_INTER_AREA);
        }
        else{
            ROS_INFO_ONCE("No rescaling necessary");
            resized_image = depth_image;
        }

        sensor_msgs::Image msg;
        std_msgs::Header header;

        pcl_conversions::fromPCL(pcl->header, header);

//        header.frame_id = virtual_cam_frame;
//        header.seq = seq++;
//        header.stamp = ros::Time(pcl->header.stamp);

        auto img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, resized_image);
        img_bridge.toImageMsg(msg);
        depth_pub.publish(msg);
        depth_image.release();
        resized_image.release();
    }

}
