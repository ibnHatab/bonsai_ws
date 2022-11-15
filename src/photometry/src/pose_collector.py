#! /usr/bin/python3

import os
import json
from  collections import OrderedDict

import rospy
import tf2_py as tf2
import tf2_ros

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from rviz_write_button.msg import WriteMsg
from jsk_rviz_plugins.msg import OverlayText
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np
from transforms3d import _euler_from_quaternion_msg


# Instantiate CvBridge
bridge = CvBridge()

text_pub = rospy.Publisher("text_rviz_hud", OverlayText, queue_size=1)


class PoseCollector:

    args_cache_time = 1.0  # seconds

    def __init__(self):

        self.target_frame = "odom"
        self.source_frame = "cam_link"
        self.lookup_offset = -0.100  # 100 ms

        self.camera_info = OrderedDict({
            "fl_x": 1019.37,
            "fl_y": 1016.04,
            "k1": 0.,
            "k2": 0.,
            "p1": 0.,
            "p2": 0.,
            "cx": 632.40,
            "cy": 490.07,
            "w": 1280,
            "h": 960,
            "aabb_scale": 16,
        })

        self.frames = []

        self.storage = None
        self.img_idx = 1
        self.folder_idx = 1
        self.folder_path = None
        self.take_image = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(PoseCollector.args_cache_time))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        try:
            self.storage = rospy.get_param('~storage')
            rospy.loginfo("Storage value: %s", self.storage)
        except KeyError:
            rospy.logerr("Storage value missing")
            raise

        self._make_scene_folder()

        rospy.Subscriber('/write_output', WriteMsg, self.shutter_cb, queue_size=1)
        rospy.Subscriber('/clicked_point', PointStamped, self.record_cb, queue_size = 1)

        image_topic = "/pylon_camera_node/image_rect"
        rospy.Subscriber(image_topic, Image, self.image_callback)

        # camera_info_topic = "/pylon_camera_node/camera_info"
        # rospy.Subscriber(camera_info_topic, Image, self.camera_info_callback)

        rospy.on_shutdown(self._save_transform)

        rospy.spin()

    def _hud_info(self, msg):
        text = OverlayText()
        text.width = 600
        text.height = 400
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = msg
        text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        text_pub.publish(text)

    def _display_hud(self, xyz, euler):
        precision = 3
        msg = "Camera transform info:"
        msg += "\n- Translation: [{:.{p}f}, {:.{p}f}, {:.{p}f}]\n".format(xyz.x,
                                                                          xyz.y,
                                                                          xyz.z,
                                                                          p=precision)
        msg += "- Rotation in RPY (degree) "
        msg += "[{:.{p}f}, {:.{p}f}, {:.{p}f}]".format(math.degrees(euler[0]),
                                                       math.degrees(euler[1]),
                                                       math.degrees(euler[2]),
                                                       p=precision)
        msg += "\n- Scene: {} / Image: {}".format(self.folder_idx, self.img_idx)

        self._hud_info(msg)

    def _get_lookup_transform(self):
        cur_time = rospy.Time.now()
        lookup_time = cur_time + rospy.Duration(self.lookup_offset)

        try:
            ts = self.tf_buffer.lookup_transform(self.source_frame,
                                                 self.target_frame,
                                                 lookup_time)
        except tf2.LookupException as ex:
            msg = "At time {}, (current time {}) ".format(lookup_time.to_sec(),
                                                          cur_time.to_sec())
            rospy.logerr(msg + str(ex))
            return None, None, None
        except tf2.ExtrapolationException as ex:
            msg = "(current time {}) ".format(cur_time.to_sec())
            rospy.logerr(msg + str(ex))
            return None, None, None

        xyz = ts.transform.translation
        quat = ts.transform.rotation

        euler, transform_matrix = _euler_from_quaternion_msg(quat)

        # import pdb
        # pdb.set_trace()
        transform_matrix[:3, 3] = [xyz.x, xyz.y, xyz.z]

        return xyz, euler, transform_matrix

    def _make_scene_folder(self):
        folder_name = 'scene_%02d'
        while True:
            self.folder_path = os.path.join(self.storage, folder_name % self.folder_idx)
            try:
                os.mkdir(self.folder_path)
                os.mkdir(os.path.join(self.folder_path, 'images'))
            except FileExistsError:
                self.folder_idx += 1
            else:
                break

    def _save_transform(self):
        file_name = os.path.join(self.folder_path, 'transforms_ros.json')
        transform = OrderedDict(self.camera_info)
        transform['frames'] = self.frames

        with open(file_name, 'w') as outfile:
            json.dump(transform, outfile, indent=4, separators=(", ", ": "), sort_keys=False)

    def shutter_cb(self, msg):
        self.take_image = True

    def record_cb(self, msg):
        print('>> record_cb ', self.folder_idx)
        self._save_transform()
        self.folder_idx += 1
        self.frames = []
        self._make_scene_folder()

    def camera_info_callback(self, msg):

        if self.camera_info:  # pragme once
            return

        (fx, fy, cx, cy) = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])
        if msg.distortion_model == "plumb_bob":
            k1, k2, t1, t2, k3 = msg.D

        k1, k2 = 0., 0.  # save rectified image
        self.camera_info = OrderedDict({
            "fl_x": fx,
            "fl_y": fy,
            "k1": k1,
            "k2": k2,
            "p1": 0.,
            "p2": 0.,
            "cx": cx,
            "cy": cy,
            "w": msg.width,
            "h": msg.height,
            "aabb_scale": 16,
        })

    def image_callback(self, msg):
        xyz, euler, transform_matrix = self._get_lookup_transform()

        if not (xyz and euler):
            return

        if not self.take_image:
            self._display_hud(xyz, euler)
            return

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as ex:
            msg = ""
            rospy.logerr(msg + str(ex))
        else:
            # Save your OpenCV2 image as a jpeg
            image_base_path = os.path.join('./images', 'frame_%04d.jpeg' % self.img_idx)
            image_path = os.path.join(self.folder_path, image_base_path)
            cv2.imwrite(image_path, cv2_img)
            self.img_idx += 1

            self.frames.append({
                "file_path": image_base_path,
                "transform_matrix": transform_matrix.tolist()
            })

        self.take_image = False

if __name__ == "__main__":
    rospy.init_node('pose_collector_node')
    rospy.loginfo(rospy.get_name() + ' start')

    try:
        rosbag_record = PoseCollector()
    except rospy.ROSInterruptException:
        pass
