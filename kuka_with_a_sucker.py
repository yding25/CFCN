# -*- coding: utf-8 -*-
# Import third-party packages 
import os, sys
import time
import math
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import pybullet
import pybullet_data
import cv2
import imageio_ffmpeg
from base64 import b64encode
from IPython.display import HTML
from google.colab.patches import cv2_imshow
from IPython.display import display
# from tqdm.notebook import tqdm
from tqdm import tqdm
from utils import Client
import random

"""# Initialize Environment"""

# initialize env 
demo = Client()

# sample object pose
pose_list = demo.sample_pose()
print(len(pose_list))

# select object 
file_object_z = open('file_object_z.txt', 'a')
path_urdf_models = '/home/yan/githubBase/CFCN/pybullet-URDF-models/urdf_models/models/'
utensil_list = os.listdir(path_urdf_models)
utensil_list_X = random.sample(utensil_list, k=16)
index = 0
for object in utensil_list_X: # find path of object's urdf
  print('{}'.format(object))
  path_object = os.path.join(path_urdf_models, object)
  for file in os.listdir(path_object):
    if file.endswith('.urdf'):
      path_object_urdf = os.path.join(path_object, file)
      print('path of object urdf: {}'.format(path_object_urdf))
      utensil_name = path_object_urdf
      utensil_pose = pose_list[index]
      print('utensil_pose: {}'.format(utensil_pose))
      utensil_id = demo.add_object(path_object_urdf, utensil_pose)
      [min_x, min_y, min_z], [max_x, max_y, max_z] = demo.get_bounding_box(utensil_id)
      # print('width:{} length:{} height:{}\n'.format(round(max_x - min_x, 2), round(max_y - min_y, 2), round(max_z - min_z, 2)))
      print('min_z: {}, max_z: {}'.format(min_z, max_z))
      file_object_z.write('{}, {}, {}\n'.format(object, min_z, max_z))
      file_object_z.flush()
      index += 1

# add objects 
utensil_id = demo.add_objects(utensil_name, utensil_pose)
print('utensil_id: {}'.format(utensil_id))

# get AABB info 
for key in utensil_id:
  print('object: {}, and its AABB:'.format(key))
  [min_x, min_y, min_z], [max_x, max_y, max_z] = demo.get_bounding_box(utensil_id[key])
  print('min_x:{} min_y:{} min_z:{}'.format(min_x, min_y, min_z))
  print('max_x:{} max_y:{} max_z:{}\n'.format(max_x, max_y, max_z))

  if 'table' in key:
    print('-'*40, '\n')
    print('       y={:.2}       y={:.2}'.format(max_y, min_y))
    print('x={:.2}   —— —— —— —— ——'.format(max_x))
    print('       |               |')
    print('       |               |')
    print('       |     table     |')
    print('       |               |')
    print('       |               |')
    print('x={:.2}   —— —— —— —— ——'.format(min_x))
    print('\n', '-'*40, '\n')

# display initial environment 
rgb, depth, mask = demo.render_image()
plt.imshow(rgb)
# plt.show()

# # start recording
# video = demo.reset_video()

# # pick-place utensils
# object_id = utensil_id['bread plate']
# object_position_init = utensil_pose['bread plate'][0]
# #please enter object's goal position, where 0.5<x<1.5, -0.75<y<0.75
# position_x = 1.0
# position_y = 0.0
# object_position_end = [position_x, position_y, 0.8]
# demo.pick_place(object_id, object_position_init, object_position_end, video)

# demo.home_joints()
# rgb, depth, mask = demo.render_image()
# demo.render_video(video, np.ascontiguousarray(rgb))
# plt.imshow(rgb) # show the last image
# # plt.show()

# video.close()
# demo.play_video()

# wait for 2 seconds
# demo.wait(500)

# disconnect pybullet 
demo.disconnect()