#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from merged_depth_cam.cfg import ModCamInfoConfig

from sensor_msgs.msg import CameraInfo


class ModCamInfo(object):
    def __init__(self):
        self.target_frame = ""
        self.pub = None
        self.target_focal_length_x = 570.
        self.target_focal_length_y = 570.
        self.target_cx = 0.0
        self.target_cy = 0.0
        self.target_width = 640
        self.target_height = 480

    def dynamic_reconfigure_cb(self, config, level):
        self.target_focal_length_x = config["fx"]
        self.target_focal_length_y = config["fy"]
        self.target_cx = config["cx"]
        self.target_cy = config["cy"]
        self.target_width = config["target_width"]
        self.target_height = config["target_height"]
        return config

    def _got_cam_info(self, info):
        k = list(info.K)
        p = list(info.P)

        if isinstance(self.target_focal_length_x, (int, float)):
            k[0] = self.target_focal_length_x
            p[0] = self.target_focal_length_x

        if isinstance(self.target_focal_length_y, (int, float)):
            k[4] = self.target_focal_length_y
            p[5] = self.target_focal_length_y

        k[2] = self.target_cx
        p[2] = self.target_cx

        k[5] = self.target_cy
        p[6] = self.target_cy

        info.K = tuple(k)
        info.P = tuple(p)

        info.width = self.target_width
        info.height = self.target_height

        info.header.frame_id = self.target_frame

        self.pub.publish(info)

    def main(self):
        rospy.init_node("mod_cam_info")

        self.target_frame = rospy.get_param("~target_frame")
        self.target_width = rospy.get_param("~target_width", 640)
        self.target_height = rospy.get_param("~target_height", 480)
        self.target_focal_length_x = rospy.get_param("~target_focal_length_x", None)
        self.target_focal_length_y = rospy.get_param("~target_focal_length_y", None)

        rospy.Subscriber("input_info", CameraInfo, self._got_cam_info)
        self.pub = rospy.Publisher("output_info", CameraInfo, queue_size=1)

        Server(ModCamInfoConfig, self.dynamic_reconfigure_cb)

        rospy.spin()


if __name__ == '__main__':
    c = ModCamInfo()
    c.main()
