// #include <pluginlib/class_list_macros.h>
#include "MergePCL.hpp"
#include "std_msgs/Header.h"
#include "tf2/convert.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>

#include <ctime>

using namespace sensor_msgs;
using namespace std_msgs;
using namespace message_filters;

namespace ap34 {
    bool transformPointCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf2_ros::Buffer &tf_buffer)
    {
        if (in.header.frame_id == target_frame)
        {
            out = in;
            return (true);
        }

        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tf_buffer.lookupTransform (target_frame, in.header.frame_id, in.header.stamp);
        }
        catch (tf::LookupException &e)
        {
            ROS_ERROR ("%s", e.what ());
            return (false);
        }
        catch (tf::ExtrapolationException &e)
        {
            ROS_ERROR ("%s", e.what ());
            return (false);
        }

        Eigen::Affine3f eigen_transform = tf2::transformToEigen(transform).cast<float>();
        pcl_ros::transformPointCloud (eigen_transform.matrix(), in, out);

        out.header.frame_id = target_frame;
        out.header.stamp = in.header.stamp;
        out.header.seq = in.header.seq;
        return (true);
    }

    MergePCL::MergePCL(ros::NodeHandle& nh) :
            tfListener(tfBuffer),
            pcl_sub_a(nh, "input_pcl_a", 1),
            pcl_sub_b(nh, "input_pcl_b", 1),
            pcl_synchronizer(PCLSyncPolicy(3), pcl_sub_a, pcl_sub_b)
    {
        this->nh = nh;

        pcl_synchronizer.registerCallback(boost::bind(&MergePCL::IncomingPCL, this, _1, _2));
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("output_merged_pcl", 1);

        usleep(500000);

        nh.param<std::string>("target_frame", target_frame, "virtual_cam_link");
    }

    void MergePCL::IncomingPCL(const sensor_msgs::PointCloud2::ConstPtr& pclA, const sensor_msgs::PointCloud2::ConstPtr& pclB) {
        auto start = ros::Time::now();

        sensor_msgs::PointCloud2 targetA, targetB;
        try{
            if(!transformPointCloud(target_frame, *pclA, targetA, tfBuffer)) return;
            if(!transformPointCloud(target_frame, *pclB, targetB, tfBuffer)) return;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.25).sleep();
            return;
        }

        auto mid1 = ros::Time::now();

        sensor_msgs::PointCloud2 ros_final;
        pcl::concatenatePointCloud(targetA, targetB, ros_final);

        auto mid2 = ros::Time::now();

        pcl_pub.publish(ros_final);

        auto end = ros::Time::now();

        ROS_INFO_STREAM_THROTTLE(1.0, "==========================");
        ROS_INFO_STREAM_THROTTLE(1.0, "Transform   : " << std::fixed << std::setw(8) << std::setfill(' ') << std::setprecision(3) << (mid1-start).nsec/1e+6 << " ms");
        ROS_INFO_STREAM_THROTTLE(1.0, "Concatenate : " << std::fixed << std::setw(8) << std::setfill(' ') << std::setprecision(3) << (mid2-mid1).nsec/1e+6 << " ms");
        ROS_INFO_STREAM_THROTTLE(1.0, "Publish     : " << std::fixed << std::setw(8) << std::setfill(' ') << std::setprecision(3) << (end-mid2).nsec/1e+6 << " ms");
        ROS_INFO_STREAM_THROTTLE(1.0, "Total       : " << std::fixed << std::setw(8) << std::setfill(' ') << std::setprecision(3) << (end-start).nsec/1e+6 << " ms");

    }
}
