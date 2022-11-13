#! /usr/bin/python3

import os

import rospy
import tf2_py as tf2
import tf2_ros

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from rviz_write_button.msg import WriteMsg
from jsk_rviz_plugins.msg import OverlayText

from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
from transforms3d import _euler_from_quaternion_msg

# Instantiate CvBridge
bridge = CvBridge()

text_pub = rospy.Publisher("text_rviz_hud", OverlayText, queue_size=1)


class PoseCollector:

    args_cache_time = 1.0  # seconds

    def __init__(self):

        self.source_frame = "odom"
        self.target_frame = "cam_link"
        self.lookup_offset = -0.100  # 100 ms

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

        folder_name = 'scene_%02d'
        while True:
            self.folder_path = os.path.join(self.storage, folder_name % self.folder_idx)
            try:
                os.mkdir(self.folder_path)
            except FileExistsError:
                self.folder_idx += 1
            else:
                break

        rospy.Subscriber('/write_output', WriteMsg, self.shutter_cb, queue_size=1)

        image_topic = "/pylon_camera_node/image_raw"
        rospy.Subscriber(image_topic, Image, self.image_callback)

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
            return
        except tf2.ExtrapolationException as ex:
            msg = "(current time {}) ".format(cur_time.to_sec())
            rospy.logerr(msg + str(ex))
            return None, None

        xyz = ts.transform.translation
        quat = ts.transform.rotation

        euler = _euler_from_quaternion_msg(quat)

        return xyz, euler

    def shutter_cb(self, msg):
        self.take_image = True

    def image_callback(self, msg):
        xyz, euler = self._get_lookup_transform()

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
            image_path = os.path.join(self.folder_path, 'camera_image_%02d.jpeg' % self.img_idx)
            cv2.imwrite(image_path, cv2_img)
            self.img_idx += 1

        self.take_image = False

if __name__ == "__main__":
    rospy.init_node('pose_collector_node')
    rospy.loginfo(rospy.get_name() + ' start')

    try:
        rosbag_record = PoseCollector()
    except rospy.ROSInterruptException:
        pass
