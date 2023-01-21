#!/usr/bin/env python

# Utilities
import os
import sys
import math
import time
import rospy
import tf
import copy
import numpy as np
import h5py
import subprocess
from geometry_msgs.msg import Pose, Point, PoseArray, WrenchStamped, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Temperature
from capsen_vision.srv import SetProperty, MoveLinkToPose
from tf.transformations import *

#####################
# Global Parameters
POSITION_NAMES = [
  'tucked_home',
  'wristup_home',
  'wristup_furnace_front',
  'wristup_furnace_pick',
  'wristup_furnace_lift',
  'wristup_furnace_withdrawn',
  'press_top',
]

class ForgingClient:

  def __init__(self):
    self._move_to_joint_positions_client = rospy.ServiceProxy(
        '/planner/set_mode_service', SetProperty)

    self._move_link_to_pose_client = rospy.ServiceProxy(
        '/planner/move_link_to_pose_service', MoveLinkToPose)

    self._set_io_client = rospy.ServiceProxy(
        '/planner/set_io_service', SetProperty)

    self._set_collision_checking_client = rospy.ServiceProxy(
        '/planner/set_collision_checking_service', SetProperty)

  def set_io(self, name, activate):
    # name must match the second column of exactly one of the rows in io_map.csv
    response = self._set_io_client(name=name, on_off=activate)

    if response.error:
      print("failed to set DIO: %s"%response.error)

  def set_collision_checking(self, link_name, allow_collision):
    response = self._set_collision_checking_client(
        name=link_name, on_off=allow_collision)

    if response.error:
      print("failed to set collision checking: %s"%response.error)
      
  def move_home(self, speed):
    """
    Parameters
    ----------
    speed : float
      A decimal value in [0,1] that commands the robot speed as a proportion of
      its max speed
    """
    response = self._move_to_joint_positions_client(type=0, abs_value=speed)

    if response.error:
      print("failed to move home: %s"%response.error)

  def move_incremental(self, dx, dy, dz):
    try:
      target_pose = self.get_current_pose()
    except ValueError as err:
      print(err.args)
      raise ValueError('move_incremental unable to determine current pose')
    
    move_speed = 0.01
    target_pose.position.x += dx
    target_pose.position.y += dy
    target_pose.position.z += dz
    try:
      self.move_scanning_object_to_pose(target_pose, move_speed)
    except ValueError as err:
      print(err.args)
      raise ValueError('move_incremental failed to move robot as expected')

  def move_to_position_names(self, speed):
    """
    Parameters
    ----------
    speed : float
      A decimal value in [0,1] that commands the robot speed as a proportion of
      its max speed
    """
    # Make a separate call to the service for each position in the list.
    for position_name in POSITION_NAMES:
      response = self._move_to_joint_positions_client(
          type=1, name=position_name, abs_value=speed)

      if response.error:
        print("failed to move to joint position: %s"%response.error)
        break

  def move_to_scanning_positions(self, speed):
    """
    Parameters
    ----------
    speed : float
      A decimal value in [0,1] that commands the robot speed as a proportion of
      its max speed
    """
    response = self._move_to_joint_positions_client(
        type=2, val_a=50, abs_value=speed)

    if response.error:
      print("failed to move to scanning positions: %s"%response.error)

  def move_scanning_object_to_pose(self, pose, speed):
    """
    Parameters
    ----------
    speed : float
      A decimal value in [0,1] that commands the robot speed as a proportion of
      its max speed

    pose : geometry_msgs/Pose.msg
    """
    response = self._move_link_to_pose_client(link="object_link", pose=pose, speed=speed)

    if response.error:
      print("failed to move object to pose: %s"%response.error)

    start_time = time.time()
    timeout = 10
    pose_mismatch = True
    pos_err_threshold = .001
    rot_err_threshold = .002
    while (time.time() - start_time) < timeout and pose_mismatch == True:
      try:
        current_pose = self.get_current_pose()
      except ValueError as err:
        print(err.args)
        raise ValueError('unable to determine current pose')

      x_err = abs(pose.position.x - current_pose.position.x)
      y_err = abs(pose.position.y - current_pose.position.y)
      z_err = abs(pose.position.z - current_pose.position.z)
      rx_err = abs(pose.orientation.x - current_pose.orientation.x)
      ry_err = abs(pose.orientation.y - current_pose.orientation.y)
      rz_err = abs(pose.orientation.z - current_pose.orientation.z)
      rw_err = abs(pose.orientation.w - current_pose.orientation.w)

      pos_err = x_err + y_err + z_err
      rot_err = rx_err + ry_err + rz_err + rw_err

      if pos_err < pos_err_threshold and rot_err < rot_err_threshold:
        pose_mismatch = False

      time.sleep(0.01)
    
    if (time.time() - start_time) >= timeout:
      raise ValueError('Timeout moving to object pose')
    else:
    #   rospy.loginfo('Successfully moved to target object pose')
    #   rospy.loginfo('Position error = {}; Rotation error = {}'.format(pos_err, rot_err))
      pass
  
  def get_current_pose(self):
    tf_listener = tf.TransformListener()
    pose = Pose()
    try:
      tf_listener.waitForTransform('/base_link','/object_link',rospy.Time(0), rospy.Duration(4.0))
      (pos, rot) = tf_listener.lookupTransform('/base_link', '/object_link', rospy.Time(0))
      pose.position.x = pos[0]
      pose.position.y = pos[1]
      pose.position.z = pos[2]
      pose.orientation.x = rot[0]
      pose.orientation.y = rot[1]
      pose.orientation.z = rot[2]
      pose.orientation.w = rot[3]
      return pose
    except:
      raise ValueError('Get_current_pose failed')


class PressSchedule():

  def __init__(self):

    self.poses = PoseArray()
    self.press_targets = []
    self.minZs = []
    self.maxZs = []

  def read_hdf5_file(self):
    with h5py.File('IncrementalFormingFullRun.hdf5', 'r') as f:
      # keepPressing_dataset = f['keepPressing']
      # if keepPressing_dataset[0] == False:
      #   rospy.loginfo("Pressing complete!")
      #   raise SystemExit
      
      pose_dataset = f['poses']
      for data in pose_dataset:
        # print("Pose from hdf5 file: ", data)

        pose = Pose()
        pose.position.x = data[0]
        pose.position.y = data[1]
        pose.position.z = data[2]
        pose.orientation.x = data[3]
        pose.orientation.y = data[4]
        pose.orientation.z = data[5]
        pose.orientation.w = data[6]
        self.poses.poses.append(copy.deepcopy(pose))
      
      press_target_dataset = f['hitMag']
      for data in press_target_dataset:
        self.press_targets.append(copy.deepcopy(data))

      maxZ_dataset = f['maxZ']
      for data in maxZ_dataset:
        self.maxZs.append(copy.deepcopy(data))
      
      minZ_dataset = f['minZ']
      for data in minZ_dataset:
        self.minZs.append(copy.deepcopy(data))

  def call_D3D(self):
    program_filepath = "/home/capsen/AIMS_D3D/DREAM3D-6.6.332.3886a295b-Linux-x86_64/bin/PipelineRunner"
    json_filepath = "/home/capsen/Downloads/D3D/IncrementalFormingPipeline-LinuxTest.json"
    subprocess.call([program_filepath, "-p", json_filepath])


def main():
  # create_file()
  # subprocess.call(["/home/capsen/AIMS_D3D/DREAM3D-6.6.332.3886a295b-Linux-x86_64/bin/PipelineRunner", "-p", "/home/capsen/Downloads/D3D/IncrementalFormingPipeline-LinuxTest.json"])

  rospy.init_node('forge_gp7_node')
  press_schedule = PressSchedule()
  press_schedule.read_hdf5_file()
  forging_client = ForgingClient()

  for index, value in enumerate(zip(press_schedule.poses.poses, press_schedule.press_targets, press_schedule.minZs, press_schedule.maxZs)):
    pose = value[0]
    press_target = value[1]
    minZ = value[2]
    maxZ = value[3]
    # print("pose #{} = {}".format(index, pose))
    # print("press_target #{} = {}".format(index, press_target))
    # print("minZ #{} = {}".format(index, minZ))
    # print("maxZ #{} = {}".format(index, maxZ))
    pose.position.z += 0.030
    forging_client.move_scanning_object_to_pose(pose, 0.02)
    pose.position.z -= 0.020
    forging_client.move_scanning_object_to_pose(pose, 0.02)
    a = 1

if __name__ == "__main__":
  main()