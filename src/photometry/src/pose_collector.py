#! /usr/bin/python3

import os
import json

import rospy
import tf2_py as tf2
import tf2_ros

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from rviz_write_button.msg import WriteMsg
from jsk_rviz_plugins.msg import OverlayText
from geometry_msgs.msg import PointStamped
from map_msgs.srv import SetMapProjections, SetMapProjectionsRequest

from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np
from collections import OrderedDict

from transforms3d import _euler_from_quaternion_msg
from recorder import Recorder
from diagnostic_msgs.msg import KeyValue

#### DEBUG
from geometry_msgs.msg import Pose, Point, Quaternion
import rviz_tools
import tf
from sensor_msgs.msg import CameraInfo

markers = rviz_tools.RvizMarkers('nerf_world', 'visualization_marker')

def rotmat2qvec(R):
    Rxx, Ryx, Rzx, Rxy, Ryy, Rzy, Rxz, Ryz, Rzz = R.flat
    K = np.array([
        [Rxx - Ryy - Rzz, 0, 0, 0],
        [Ryx + Rxy, Ryy - Rxx - Rzz, 0, 0],
        [Rzx + Rxz, Rzy + Ryz, Rzz - Rxx - Ryy, 0],
        [Ryz - Rzy, Rzx - Rxz, Rxy - Ryx, Rxx + Ryy + Rzz]]) / 3.0
    eigvals, eigvecs = np.linalg.eigh(K)
    qvec = eigvecs[[3, 0, 1, 2], np.argmax(eigvals)]
    if qvec[0] < 0:
        qvec *= -1
    return qvec
####

# Instantiate CvBridge
bridge = CvBridge()

text_pub = rospy.Publisher("text_rviz_hud", OverlayText, queue_size=1)


class PoseCollector:
    args_cache_time = 0.3  # seconds
    camera_info = OrderedDict({
        "fl_x": 1008.785600703726,
        "fl_y": 1008.153829014085,
        "cx": 620.725352114218,
        "cy": 486.3101299756897,
        "w": 1280,
        "h": 960,
        "camera_model": "OPENCV",
        "k1": -0.4131778255776392,
        "k2": 0.1488336060172014,
        "p1": -0.0002996896647241871,
        "p2": -0.0005385604028147197,
        "aabb_scale": 16,
    })

    def __init__(self):

        self.source_frame = "nerf_world"
        self.target_frame = "nerf_cam"

        self.lookup_offset = -0.130  # watch 30 hz overlap OK (yaw -2')
        self.viewer = False
        self.recorder = Recorder(subscribe=False)

        self.xs_status = {}
        self.camera_info = None
        self.frames = []
        self.storage = None
        self.img_idx = 0
        self.folder_idx = 1
        self.folder_path = None
        self.take_image = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(PoseCollector.args_cache_time))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        try:
            self.viewer = rospy.get_param('~viewer')
        except KeyError:
            pass

        try:
            self.storage = rospy.get_param('~storage')
            rospy.loginfo("Storage value: %s", self.storage)
        except KeyError:
            rospy.logerr("Storage value missing")
            raise
        self._make_scene_folder()

        rospy.Subscriber('/write_output', WriteMsg, self.shutter_cb, queue_size=1)
        rospy.Subscriber('/clicked_point', PointStamped, self.record_cb, queue_size = 1)
        rospy.Subscriber('/status', KeyValue, self.xsens_status_cb)

        rospy.Subscriber('/pylon_camera_node/image_raw', Image, self.image_callback)
        rospy.Subscriber('/pylon_camera_node/camera_info', CameraInfo, self.camera_info_callback)

        rospy.on_shutdown(self._save_transform)

        rospy.spin()

    def xsens_status_cb(self, msg):
        dw = int(msg.value, 2)

        self.xs_status = {
            'GpsValid': bool(dw & 0x04),
            'RtkStatus': bool(dw & 0x18000000),
            'FilterMode': bool(dw & 0x03800000),
        }

    def _hud_info(self, msg):
        text = OverlayText()
        text.width = 550
        text.height = 180
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
        msg += "\n"
        for name, st in self.xs_status.items():
            if st:
                msg += ' / <span style="color: green;">%s</span> ' % name
            else:
                msg += ' / <span style="color: red;">%s</span> ' % name
        msg += "\n- Scene: {} / Image: {}".format(self.folder_idx, self.img_idx)

        self._hud_info(msg)

    def _get_lookup_transform(self, stamp=0):

        exposure = rospy.Duration(.015411)
        cam0_to_imu0_time = rospy.Duration(-0.17469761937472594)
        timeshift = cam0_to_imu0_time + exposure
        timeshift = cam0_to_imu0_time

        cur_time = rospy.Time.now()
        # print(cur_time, stamp, cur_time-stamp)
        if stamp:
            lookup_time = stamp + timeshift
        else:
            lookup_time = cur_time + rospy.Duration(self.lookup_offset)

        try:

            ts = self.tf_buffer.lookup_transform(self.source_frame,
                                                 self.target_frame,
                                                 lookup_time)
        except tf2.LookupException as ex:
            msg = "At time {}, (current time {}) ".format(lookup_time.to_sec(),
                                                          cur_time.to_sec())
            rospy.logerr_once(msg + str(ex))
            return Point(), np.zeros(3), np.eye(4)

        except tf2.ExtrapolationException as ex:
            msg = "(current time {}) ".format(cur_time.to_sec())
            rospy.logerr_once(msg + str(ex))
            return Point(), np.zeros(3), np.eye(4)

        xyz = ts.transform.translation
        quat = ts.transform.rotation

        euler, transform_matrix = _euler_from_quaternion_msg(quat)

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
        file_name = os.path.join(self.folder_path, 'transforms.json')
        if self.camera_info:
            transform = OrderedDict(self.camera_info)
        else:
            transform = OrderedDict(PoseCollector.camera_info)
        transform['frames'] = self.frames

        with open(file_name, 'w') as outfile:
            json.dump(transform, outfile, indent=4, separators=(", ", ": "), sort_keys=False)

    def shutter_cb(self, msg):

        if self.img_idx == 0 and not self.viewer:
            rospy.wait_for_service('geonav_sat_fix')
            try:
                geonav_sat_fix = rospy.ServiceProxy('geonav_sat_fix', SetMapProjections)
                _ = geonav_sat_fix(SetMapProjectionsRequest())
            except rospy.ServiceException as e:
                rospy.logerr ("Service call failed: %s"%e)
                return

            self.recorder.switch_on()
            self.img_idx += 1
            return

        self.take_image = True

    def record_cb(self, msg):
        print('>> record_cb: ', self.folder_idx)
        self._save_transform()
        self.folder_idx += 1
        self.frames = []
        self.img_idx = 0
        self.recorder.switch_off()
        self._make_scene_folder()

    def camera_info_callback(self, msg):

        if self.camera_info:  # pragme once
            return

        (fx, fy, cx, cy) = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])
        k1, k2, t1, t2, k3 = msg.D

        self.camera_info = OrderedDict({
            "fl_x": fx,
            "fl_y": fy,
            "k1": k1,
            "k2": k2,
            "p1": t1,
            "p2": t2,
            "cx": cx,
            "cy": cy,
            "w": msg.width,
            "h": msg.height,
            "aabb_scale": 16,
        })
        print('Using CameraInfo:', self.camera_info)

    def image_callback(self, msg):
        xyz, euler, transform_matrix = self._get_lookup_transform(msg.header.stamp)

        self._display_hud(xyz, euler)

        if not self.take_image:
            return

        ###  DEBUG cam_world
        quat = tf.transformations.quaternion_from_euler(*euler)
        P = Pose(Point(*transform_matrix[:3, 3]), Quaternion(*quat))
        markers.publishAxis(P, 1, 0.1, 0)
        #####

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
