#! /usr/bin/python3

import os
import json

import rospy
import tf2_py as tf2
import tf2_ros

from std_msgs.msg import ColorRGBA
from jsk_rviz_plugins.msg import OverlayText
from diagnostic_msgs.msg import KeyValue

class Display:

    def __init__(self):
        self.source_frame = "cam_world"
        self.target_frame = "cam_frame"
        self.xs_status = {}
        self.lookup_offset = -0.100  # 100 ms

        camera_info_topic = "/status"
        rospy.Subscriber(camera_info_topic, KeyValue, self.xsens_status_cb)

    def xsens_status_cb(self, msg):
        dw = int(msg.value, 2)

        self.xs_status = {
            'GpsValid': bool(dw & 0x04),
            'RtkStatus': bool(dw & 0x18000000),
        }
