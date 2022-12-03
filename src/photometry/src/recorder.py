#! /usr/bin/python3
import os
import rospy
import subprocess
import signal

from geometry_msgs.msg import PointStamped


class Recorder:

    def __init__(self, subscribe=True):
        self.p = None
        self.on_switch = False
        self.storage = None

        rospy.on_shutdown(self.switch_off)

        if rospy.has_param('~storage'):
            self.storage = rospy.get_param('~storage')
            print('Recorder ready >>', self.storage)

        if subscribe:
            rospy.Subscriber('/clicked_point', PointStamped, self.record_cb, queue_size = 1)

    def switch_off(self):
        print('>> switch_off')
        self.on_switch = False

        if self.p:
            self.p.send_signal(signal.SIGTERM)
            self.p = None
            rospy.sleep(3)

    def switch_on(self):
        print('>> switch_on')
        self.on_switch = True

        camera_topic = [
            # '/clicked_point',
            # '/diagnostics',
            '/filter/free_acceleration',
            '/filter/positionlla',
            '/filter/quaternion',
            '/filter/twist',
            '/filter/velocity',
            '/gnss',
            '/imu/acceleration',
            '/imu/angular_velocity',
            '/imu/data',
            '/imu/dq',
            '/imu/dv',
            '/imu/mag',
            '/imu/time_ref',
            '/initialpose',
            '/move_base_simple/goal',
            '/pressure',
            '/pylon_camera_node/camera_info',
            # '/pylon_camera_node/currentParams',
            # '/pylon_camera_node/grab_images_raw/cancel',
            # '/pylon_camera_node/grab_images_raw/feedback',
            # '/pylon_camera_node/grab_images_raw/goal',
            # '/pylon_camera_node/grab_images_raw/result',
            # '/pylon_camera_node/grab_images_raw/status',
            # '/pylon_camera_node/grab_images_rect/cancel',
            # '/pylon_camera_node/grab_images_rect/feedback',
            # '/pylon_camera_node/grab_images_rect/goal',
            # '/pylon_camera_node/grab_images_rect/result',
            # '/pylon_camera_node/grab_images_rect/status',
            '/pylon_camera_node/image_raw',
            # '/pylon_camera_node/image_raw/compressed',
            # '/pylon_camera_node/image_raw/compressed/parameter_descriptions',
            # '/pylon_camera_node/image_raw/compressed/parameter_updates',
            # '/pylon_camera_node/image_raw/compressedDepth',
            # '/pylon_camera_node/image_raw/compressedDepth/parameter_descriptions',
            # '/pylon_camera_node/image_raw/compressedDepth/parameter_updates',
            # '/pylon_camera_node/image_raw/theora',
            # '/pylon_camera_node/image_raw/theora/parameter_descriptions',
            # '/pylon_camera_node/image_raw/theora/parameter_updates',
            '/pylon_camera_node/image_rect',
            # '/pylon_camera_node/image_rect/compressed',
            # '/pylon_camera_node/image_rect/compressed/parameter_descriptions',
            # '/pylon_camera_node/image_rect/compressed/parameter_updates',
            # '/pylon_camera_node/image_rect/compressedDepth',
            # '/pylon_camera_node/image_rect/compressedDepth/parameter_descriptions',
            # '/pylon_camera_node/image_rect/compressedDepth/parameter_updates',
            # '/pylon_camera_node/image_rect/theora',
            # '/pylon_camera_node/image_rect/theora/parameter_descriptions',
            # '/pylon_camera_node/image_rect/theora/parameter_updates',
            '/pylon_camera_node/status',
            # '/rosout',
            # '/rosout_agg',
            '/status',
            '/temperature',
            '/tf',
            '/tf_static',
            '/write_output',
        ]
        bag_path = self.storage

        command = '/opt/ros/noetic/bin/rosbag record  -o %s/nerf %s' % \
            (bag_path, ' '.join(camera_topic))

        self.p = subprocess.Popen(command,
                                  stdin=subprocess.PIPE,
                                  shell=True,
                                  executable='/bin/bash')

    def record_cb(self, msg):
        """
        start/stop.

        recording.
        """
        if not self.on_switch:
            self.switch_on()
        else:
            self.switch_off()


if __name__ == "__main__":
    rospy.init_node('recorder_node')
    rospy.loginfo(rospy.get_name() + ' start')

    try:
        rosbag_record = Recorder()
        # Wait for shutdown signal to close rosbag record
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
