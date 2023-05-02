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

# Utils
class Client():
  def __init__(self):
    pybullet.connect(pybullet.DIRECT) # pybullet.GUI for local GUI.
    # pybullet.connect(pybullet.GUI) # pybullet.GUI for local GUI.
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, -9.8)
    
    # reset robot
    self.plane_id = pybullet.loadURDF("plane.urdf")
    self.robot_id = pybullet.loadURDF("kuka_iiwa/model_vr_limits.urdf", globalScaling=1.5, basePosition=[1.7, 0.0, 1.2], baseOrientation=[0.0, 0.0, 0.0, 1.0])
    jointPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for jointIndex in range(pybullet.getNumJoints(self.robot_id)):
        pybullet.resetJointState(self.robot_id, jointIndex, jointPositions[jointIndex])
        pybullet.setJointMotorControl2(self.robot_id, jointIndex, pybullet.POSITION_CONTROL, jointPositions[jointIndex], 0)

    # camera width and height
    self.cam_width = 480
    self.cam_height = 480

  def render_image(self):
    # camera parameters
    cam_target_pos = [1.0, 0.0, 0.78]
    cam_distance = 1.5
    cam_yaw, cam_pitch, cam_roll = -90, -90, 0
    cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60
    cam_view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
    cam_projection_matrix = pybullet.computeProjectionMatrixFOV(cam_fov, self.cam_width*1./self.cam_height, cam_near_plane, cam_far_plane)
    znear, zfar = 0.01, 10.

    # get raw data
    _, _, color, depth, segment = pybullet.getCameraImage(
        width=self.cam_width,
        height=self.cam_height,
        viewMatrix=cam_view_matrix,
        projectionMatrix=cam_projection_matrix,
        shadow=1,
        flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
        renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

    # get color image.
    color_image_size = (self.cam_width, self.cam_height, 4)
    color = np.array(color, dtype=np.uint8).reshape(color_image_size)
    color = color[:, :, :3]  # remove alpha channel

    # get depth image.
    depth_image_size = (self.cam_width, self.cam_height)
    zbuffer = np.float32(depth).reshape(depth_image_size)
    depth = (zfar + znear - (2 * zbuffer - 1) * (zfar - znear))
    depth = (2 * znear * zfar) / depth

    # get segment image.
    segment = np.reshape(segment, [self.cam_width, self.cam_height]) * 1. / 255.
    return color, depth, segment

  def reset_video(self):
    video = imageio_ffmpeg.write_frames('video.mp4', (self.cam_width, self.cam_height), fps=60)
    video.send(None) # seed the video writer with a blank frame
    return video

  def render_video(self, video, image):
    video.send(np.ascontiguousarray(image))
    
  def play_video(self):
    mp4 = open('video.mp4', 'rb').read()
    data_url = "data:video/mp4;base64," + b64encode(mp4).decode()
    return HTML('<video width=480 controls><source src="%s" type="video/mp4"></video>' % data_url)

  def add_objects(self, utensil_name, utensil_pose):
    utensil_id = {}

    flags = pybullet.URDF_USE_INERTIA_FROM_FILE
    path = '/home/yan/githubBase/CFCN/urdf_models/'

    if not os.path.exists(path):
      print('Error: cannot find /home/yan/githubBase/CFCN/urdf_models/!')
      sys.exit(1)
      
    # add table
    # table_id = pybullet.loadURDF("/home/yan/githubBase/CFCN/urdf_models/furniture_table_rectangle/table.urdf", basePosition=[1.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
    table_id = pybullet.loadURDF("/home/yan/githubBase/CFCN/urdf_models/furniture_table_square/table.urdf", globalScaling=2.0, basePosition=[1.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
    utensil_id['table'] = table_id

    # add objects according to utensil_name
    if 'bread plate' in utensil_name:
      utensil_id['bread plate'] = pybullet.loadURDF(path + 'utensil_plate_circle_blue' + '/model.urdf', basePosition=utensil_pose['bread plate'][0], baseOrientation=utensil_pose['bread plate'][1], flags=flags)
    if 'butter knife' in utensil_name:
      utensil_id['butter knife'] = pybullet.loadURDF(path + 'utensil_knife_blue' + '/model.urdf', basePosition=utensil_pose['butter knife'][0], baseOrientation=utensil_pose['butter knife'][1], flags=flags)
    if 'fork' in utensil_name:
      utensil_id['fork'] = pybullet.loadURDF(path + 'utensil_fork_blue' + '/model.urdf', basePosition=utensil_pose['fork'][0], baseOrientation=utensil_pose['fork'][1],flags=flags)
    if 'spoon' in utensil_name:
      utensil_id['spoon'] = pybullet.loadURDF(path + 'utensil_spoon_blue' + '/model.urdf', basePosition=utensil_pose['spoon'][0], baseOrientation=utensil_pose['spoon'][1], flags=flags)
    if 'water glass' in utensil_name:
      utensil_id['water glass'] = pybullet.loadURDF(path + 'utensil_cup_blue' + '/model.urdf', basePosition=utensil_pose['water glass'][0], baseOrientation=utensil_pose['water glass'][1], flags=flags)
    if 'wine glass' in utensil_name:
      utensil_id['wine glass'] = pybullet.loadURDF(path + 'utensil_cup_blue' + '/model.urdf', basePosition=utensil_pose['wine glass'][0], baseOrientation=utensil_pose['wine glass'][1], flags=flags)
    if 'napkin' in utensil_name: # TO DO
      utensil_id['napkin'] = pybullet.loadURDF(path + 'utensil_cup_green' + '/model.urdf', basePosition=utensil_pose['napkin'][0], baseOrientation=utensil_pose['napkin'][1], flags=flags)
    if 'dinner knife' in utensil_name:
      utensil_id['dinner knife'] = pybullet.loadURDF(path + 'utensil_knife_blue' + '/model.urdf', basePosition=utensil_pose['dinner knife'][0], baseOrientation=utensil_pose['dinner knife'][1], flags=flags)
    if 'dinner plate' in utensil_name: # TO DO
      utensil_id['dinner plate'] = pybullet.loadURDF(path + 'utensil_plate_circle_blue_big' + '/model.urdf', basePosition=utensil_pose['dinner plate'][0], baseOrientation=utensil_pose['dinner plate'][1], flags=flags)
    if 'test' in utensil_name:
      utensil_id['test'] = pybullet.loadURDF(path + 'utensil_nerf_bowl' + '/model.urdf', basePosition=utensil_pose['test'][0], baseOrientation=utensil_pose['test'][1], flags=flags)
    return utensil_id
  
  def add_object(self, utensil_name, utensil_pose):
    flags = pybullet.URDF_USE_INERTIA_FROM_FILE
    utensil_id = pybullet.loadURDF(utensil_name, basePosition=utensil_pose[0], baseOrientation=utensil_pose[1], flags=flags)
    return utensil_id

  def get_bounding_box(self, obj_id):
    (min_x, min_y, min_z), (max_x, max_y, max_z)= pybullet.getAABB(obj_id)
    return [min_x, min_y, min_z], [max_x, max_y, max_z]

  def home_joints(self):
    jointPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for jointIndex in range(pybullet.getNumJoints(self.robot_id)):
        pybullet.resetJointState(self.robot_id, jointIndex, jointPositions[jointIndex])
        pybullet.setJointMotorControl2(self.robot_id, jointIndex, pybullet.POSITION_CONTROL, jointPositions[jointIndex], 0)

  def pick_place(self, object_id, object_position_init, object_position_end, video):
    gripper_id = None
    num_joints = pybullet.getNumJoints(self.robot_id)
    end_effector_index = 6
    # target_position = [0.9, -0.6, 0.65]
    target_position = [object_position_init[0], object_position_init[1], object_position_init[2] + 0.1]

    for step in tqdm(range(900)):
      if step % 4 == 0: # PyBullet default simulation time step is 240fps, but we want to record video at 60fps.
        rgb, depth, mask = self.render_image()
        self.render_video(video, np.ascontiguousarray(rgb))

      target_orientation = pybullet.getQuaternionFromEuler([0, 1.01*math.pi, 0])
      gripper_status = {'ungrasp': 0, 'grasp': 1}
      gripper_value = gripper_status['ungrasp']
      if step >= 150 and step < 250:
          target_position = [object_position_init[0], object_position_init[1], object_position_init[2] + 0.1] # grab object
          gripper_value = gripper_status['grasp']
      elif step >= 250 and step < 400:
          # target_position = [0.85, -0.2, 0.7 + 0.2*(step-250)/150.] # move up after picking object
          target_position = [object_position_init[0], object_position_init[1], object_position_init[2] + 0.3]
          gripper_value = gripper_status['grasp']
      elif step >= 400 and step < 600:
          # target_position = [0.85, -0.2 + 0.4*(step-400)/200., 0.9] # move to target position
          target_position = [object_position_init[0] + (object_position_end[0] - object_position_init[0]) * (step-400)/200, object_position_init[1] + (object_position_end[1] - object_position_init[1]) * (step-400)/200, object_position_init[2] + 0.3]
          gripper_value = gripper_status['grasp']
      elif step >= 600 and step < 700:
          target_position = [object_position_end[0], object_position_end[1], object_position_end[2] + 0.3] # stop at target position
          gripper_value = gripper_status['grasp']
      elif step >= 700:
          target_position = [object_position_end[0], object_position_end[1], object_position_end[2] + 0.3] # drop object
          gripper_value = gripper_status['ungrasp']

      joint_poses = pybullet.calculateInverseKinematics(self.robot_id, end_effector_index, target_position, target_orientation)
      for joint_index in range(num_joints):
          pybullet.setJointMotorControl2(bodyIndex = self.robot_id, jointIndex = joint_index, controlMode = pybullet.POSITION_CONTROL, targetPosition = joint_poses[joint_index])

      if gripper_value == 0 and gripper_id != None:
          pybullet.removeConstraint(gripper_id)
          gripper_id = None
      if gripper_value == 1 and gripper_id == None:
          cube_orn = pybullet.getQuaternionFromEuler([0, math.pi, 0])
          gripper_id = pybullet.createConstraint(self.robot_id, end_effector_index, object_id, -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0], childFrameOrientation=cube_orn)

      pybullet.stepSimulation()

  def disconnect(self):
    pybullet.disconnect()
  
  def sample_pose(self):
    # max_pose = [[1.5, 0.75, 0.6], [0, 0, 0, 1]]
    # min_pose = [[0.5, -0.75, 0.6], [0, 0, 0, 1]]
    pose_list = []
    for i in range(4):
      for j in range(4):
        temp = [[1.35 - 0.2 * i, 0.35 - 0.2 * j, 1.35], [0, 0, 0, 1]]
        pose_list.append(temp)
    return pose_list

  def wait(self, seconds, step_duration=0.01):
    num_steps = int(seconds / step_duration)
    for _ in range(num_steps):
      # pybullet.stepSimulation()
      time.sleep(step_duration)