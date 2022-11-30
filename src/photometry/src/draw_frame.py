#! /usr/bin/python3

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from jsk_rviz_plugins.msg import OverlayText
from geometry_msgs.msg import Pose, Point, Quaternion

import rviz_tools
import tf
import tf2_py as tf2
import tf2_ros

import cv2
import math
from transforms3d import _euler_from_quaternion_msg
from cv_bridge import CvBridge

cv_bridge = CvBridge()


class DrawFrame:

    def __init__(self):
        self.source_frame = "world"
        self.target_frame = "imu_link"
        self.lookup_offset = -0.100  # 100 ms

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(0.3))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.image_pub = rospy.Publisher("image_frame", Image, queue_size=1)
        self.markers = rviz_tools.RvizMarkers('world', 'visualization_marker')
        self.text_pub = rospy.Publisher("text_rviz_hud", OverlayText, queue_size=1)

        rospy.Subscriber("/pylon_camera_node/image_rect", Image, self.image_callback, queue_size=1)

    def image_callback(self, msg: Image):
        cv_img = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, channels = cv_img.shape
        cv2.circle(cv_img, (width // 2, height // 2), 10, 255, thickness=3)

        image_message = cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        self.image_pub.publish(image_message)

        xyz, euler, transform_matrix = self._get_lookup_transform()

        if not (xyz and euler):
            return

        precision = 3
        msg = "- Rotation in RPY (degree) \n"
        msg += "[{:.{p}f}, {:.{p}f}, {:.{p}f}]".format(math.degrees(euler[0]),
                                                       math.degrees(euler[1]),
                                                       math.degrees(euler[2]),
                                                       p=precision)
        self._hud_info(msg)


    def _hud_info(self, msg):
        text = OverlayText()
        text.width = 600
        text.height = 100
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = msg
        text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        self.text_pub.publish(text)

    def _get_lookup_transform(self, inverse=False):
        cur_time = rospy.Time.now()
        lookup_time = cur_time + rospy.Duration(self.lookup_offset)

        try:
            if inverse:
                ts = self.tf_buffer.lookup_transform(self.target_frame,
                                                     self.source_frame,
                                                     lookup_time)
            else:
                ts = self.tf_buffer.lookup_transform(self.source_frame,
                                                     self.target_frame,
                                                     lookup_time)
        except tf2.LookupException as ex:
            msg = "At time {}, (current time {}) ".format(lookup_time.to_sec(),
                                                          cur_time.to_sec())
            rospy.logerr_once(msg + str(ex))
            return None, None, None

        except tf2.ExtrapolationException as ex:
            msg = "(current time {}) ".format(cur_time.to_sec())
            rospy.logerr_once(msg + str(ex))
            return None, None, None

        xyz = ts.transform.translation
        quat = ts.transform.rotation

        euler, transform_matrix = _euler_from_quaternion_msg(quat)

        transform_matrix[:3, 3] = [xyz.x, xyz.y, xyz.z]

        return xyz, euler, transform_matrix

if __name__ == "__main__":
    rospy.init_node("draw_frame")
    try:
        _drawer = DrawFrame()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
