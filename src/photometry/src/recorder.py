#! /usr/bin/python3

import rospy
import subprocess, signal
import os
import RPi.GPIO as GPIO
import time

from sensor_msgs.msg import Joy
from dji_osdk_ros.srv import SetupCameraStream, SetupCameraStreamRequest
from dji_osdk_ros.srv import StereoVGASubscription, StereoVGASubscriptionRequest

class MobileComm:

    def __init__(self):
        self.p = None
        self.on_switch = False
        self.storage = None
        self.camera = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18,GPIO.OUT)
        GPIO.output(18,GPIO.LOW)

        rospy.on_shutdown(self.switch_off)


        if rospy.has_param('~storage'):

            self.storage = rospy.get_param('~storage')
            self.camera_type = rospy.get_param('~camera')


            if self.camera_type == 'vga':
                camera_topic = '/stereo_vga_subscription'
            else:
                camera_topic = '/setup_camera_stream'

            rospy.wait_for_service(camera_topic)

            if self.camera_type == 'vga':
                self.camera = rospy.ServiceProxy(camera_topic, StereoVGASubscription)
            else:
                self.camera = rospy.ServiceProxy(camera_topic, SetupCameraStream)

            print('OK >>', self.camera)
            self.led_error() # signal ready

            rc_topic = '/dji_osdk_ros/rc'
            rospy.Subscriber(rc_topic, Joy, self.joy_cb, queue_size = 1)

            # Wait for shutdown signal to close rosbag record
            rospy.spin()
        else:
            rospy.signal_shutdown(rospy.get_name() + ' no record ~storage folder specified.')

    def have_space(self):
        statvfs = os.statvfs('/')
        if (statvfs.f_frsize * statvfs.f_bavail) > 1e9: # 1Gb
            return True
        else:
            print("ERROR: storage left less than 1Gb")
            return False

    def led_error(self):
        for i in range(3):
            GPIO.output(18,GPIO.HIGH)
            time.sleep(1)
            GPIO.output(18,GPIO.LOW)



    def switch_off(self):
        print('switch_off')
        self.on_switch = False
        if self.p:
            self.p.send_signal(signal.SIGTERM)
            self.p = None

        if self.camera_type == 'vga':
            msg = StereoVGASubscriptionRequest()
            msg.vga_freq = 1
            msg.front_vga = 1
            msg.unsubscribe_vga = 1
        else:
            msg = SetupCameraStreamRequest()
            msg.cameraType = 0
            msg.start = 0

        ret = self.camera(msg)
        print('camoff: ', ret)
        GPIO.output(18,GPIO.LOW)

        rospy.sleep(3)

    def switch_on(self):
        print('switch_on')
        self.on_switch = True

        if self.camera_type == 'vga':
            msg = StereoVGASubscriptionRequest()
            # msg.vga_freq = 1
            msg.front_vga = 1
            msg.unsubscribe_vga = 0
            print('switch_on vga', msg)

        else:
            msg = SetupCameraStreamRequest()
            msg.cameraType = 0
            msg.start = 1

        ret = self.camera(msg)
        print('camon: ', ret, self.camera_type)

        if self.camera_type == 'vga':
            camera_topic = '/dji_osdk_ros/stereo_vga_front_left_images'
        else:
            camera_topic = '/dji_osdk_ros/fpv_camera_images'

        # --lz4 /camera /dji_osdk_ros/fpv_camera_images
        # /dji_osdk_ros/stereo_vga_front_left_images

        command = '/opt/ros/melodic/bin/rosbag record  -o %s/livox_dji /livox/lidar /livox/imu /dji_osdk_ros/imu %s' % (self.storage, camera_topic)

        if self.have_space():
            GPIO.output(18,GPIO.HIGH)
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')
        else:
            self.p = None
            self.led_error()

    def joy_cb(self, msg):

        if msg.axes[5] >= 10000.0 and self.on_switch == False:
            self.switch_on()

        if msg.axes[5] <= -10000.0 and self.on_switch == True:
            self.switch_off()


if __name__ == "__main__":
    rospy.init_node('mobile_comm_node')
    rospy.loginfo(rospy.get_name() + ' start')

    try:
        rosbag_record = MobileComm()
    except rospy.ROSInterruptException:
        pass
