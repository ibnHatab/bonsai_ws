#! env python3

import json
import math
import numpy as np
import sys

from transforms3d import _euler_from_matrix
import rospy
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, Point, Quaternion
import rviz_tools

rospy.init_node('recorder_node')
markers = rviz_tools.RvizMarkers('cam_world', 'visualization_marker')

nerf_files = ['transforms.json']

if len(sys.argv) >= 2:
    nerf_files = sys.argv[1:]

for nerf_file in nerf_files:
    with open(nerf_file, 'r') as infile:
        transform_nerf = json.load(infile)


    precision = 3
    for frame in transform_nerf['frames']:
        pass
        file_path = frame['file_path']
        transform_matrix = np.array(frame['transform_matrix'])

        xyz = transform_matrix[:3, 3]
        rotation = transform_matrix[:3, :3]

        euler = _euler_from_matrix(rotation)
        quat = quaternion_from_euler(*euler)


        P = Pose(Point(*xyz), Quaternion(*quat))
        axis_length = 1
        axis_radius = 0.1
        markers.publishAxis(P, axis_length, axis_radius, 0) # pose, axis length, radius, lifetime
        rospy.sleep(0.2)


        msg = ""
        msg += "- Image: {}".format(file_path)
        msg += "- Translation: [{:.{p}f}, {:.{p}f}, {:.{p}f}]".format(xyz[0],
                                                                      xyz[1],
                                                                      xyz[2],
                                                                      p=precision)
        msg += " - Rotation in RPY (degree) "
        msg += "[{:.{p}f}, {:.{p}f}, {:.{p}f}]".format(math.degrees(euler[0]),
                                                       math.degrees(euler[1]),
                                                       math.degrees(euler[2]),
                                                       p=precision)
        print(msg)
