#! /usr/bin/python3

import rospy
import tf2_py as tf2
import math

from std_msgs.msg import ColorRGBA
from jsk_rviz_plugins.msg import OverlayText
from diagnostic_msgs.msg import KeyValue
from transforms3d import _euler_from_quaternion_msg

class Display:

    def __init__(self):
        self.source_frame = "cam_world"
        self.target_frame = "cam_frame"
        self.xs_status = {}
        self.lookup_offset = -0.100  # 100 ms
        self.stop = False
        self.text_pub = rospy.Publisher("text_rviz_hud", OverlayText, queue_size=1)

        camera_info_topic = "/status"
        rospy.Subscriber(camera_info_topic, KeyValue, self.xsens_status_cb)

    def xsens_status_cb(self, msg):
        dw = int(msg.value, 2)

        self.xs_status = {
            'GpsValid': bool(dw & 0x04),
            'RtkStatus': bool(dw & 0x18000000),
        }

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
        self.text_pub.publish(text)

    def _display_hud(self, xyz, euler, status):
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

        msg += "\n"
        for name, st in status.items():
            if st:
                msg += '<span style="color: green;">%s</span> ' % name
            else:
                msg += '<span style="color: red;">%s</span> ' % name

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

    def run(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            xyz, euler, transform_matrix = self._get_lookup_transform()

            if not (xyz and euler):
                continue

            self._display_hud(xyz, euler, self.xs_status)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('rviz_display_node')
    try:
        client = Display()
        # Wait for shutdown signal to close rosbag record
        client.run()
    except rospy.ROSInterruptException:
        pass
