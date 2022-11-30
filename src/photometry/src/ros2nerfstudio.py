#! env python3

import json
from collections import OrderedDict
import numpy as np
import sys

tf_file_from = '/mnt/storage/scene_172/transforms.json'
tf_file_to = 'transforms.json'

if len(sys.argv) == 3:
    tf_file_from = sys.argv[1]
    tf_file_to = sys.argv[2]

with open(tf_file_from, 'r') as infile:
    transform_from = json.load(infile)

transform_to = OrderedDict(transform_from)

transform_to['frames'] = []
# rotor = np.array([
#     [0, -1, 0],
#     [0,  0, 1],
#     [-1, 0, 0],
# ])
# # rotor = np.array([
# #     [1, 0, 0],
# #     [0,  1, 0],
# #     [0, 0, 1],
# # ])
# # rotor = np.array([
# #     [0, 0, 1],
# #     [1, 0, 0],
# #     [0, 1, 0],
# # ])

# # rotor = np.array([
# #     [ 0,  0, -1],
# #     [-1,  0,  0],
# #     [ 0,  1,  0],
# # ])
# narotor = np.array([
#     [1,  0,  0, 0],
#     [0,  1,  0, 0],
#     [0,  0,  1, 0],
#     [0,  0,  0, 1],
# ])
# transform_matrix[:3, 3] *= -1
# # transform_matrix[:3, 3] = rotor @ transform_matrix[:3, 3]
# transform_matrix[:3, :3] = (rotor @ (transform_matrix[:3, :3]).T).T

for frame in transform_from['frames']:
    pass
    name = frame['file_path']
    transform_matrix = np.array(frame['transform_matrix'])

    rotation = transform_matrix[:3, :3]  # qvec2rotmat(im_data.qvec)
    translation = transform_matrix[:3, 3]  # im_data.tvec.reshape(3, 1)
    translation = translation.reshape(3, 1)

    w2c = np.concatenate([rotation, translation], 1)
    w2c = np.concatenate([w2c, np.array([[0, 0, 0, 1]])], 0)
    c2w = np.linalg.inv(w2c)
    # Convert from COLMAP's camera coordinate system to ours
    c2w[0:3, 1:3] *= -1
    c2w = c2w[np.array([1, 0, 2, 3]), :]
    c2w[2, :] *= -1

    frame = {
        "file_path": name,
        "transform_matrix": c2w.tolist(),
    }
    transform_to['frames'].append(frame)

    # narotor[:3, 3] = rotor @ transform_matrix[:3, 3]
    # transform_to['frames'].append({'file_path': file_path,
    #                                'transform_matrix': transform_matrix.tolist()})

with open(tf_file_to, 'w') as outfile:
    json.dump(transform_to, outfile, indent=4, separators=(", ", ": "), sort_keys=False)
