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
from geometry_msgs.msg import Pose, Point, PoseArray, WrenchStamped, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Temperature
from capsen_vision.srv import SetProperty, MoveLinkToPose
from tf.transformations import *

class PressSchedule():
  def __init__(self):
    # init to schedule 1 by default
    self.set_schedule_1()

  def set_schedule_1(self):

    self.poses = PoseArray()

    # pose1 is the first corner
    pose1 = Pose()
    pose1.position.x = 0.577
    pose1.position.y = -0.205
    pose1.position.z = 0.502
    pose1.orientation.x = -0.3927
    pose1.orientation.y = 0
    pose1.orientation.z = 0
    pose1.orientation.w = 0.91965

    step_dist = -0.015
    num_steps = 3

    press_amount = 0.005
    self.poses.poses = [pose1]
    # NB: press_targets are either incremental or desired spacing between upper & lower dies
    self.incrementalFlag = [True]
    self.press_targets = [press_amount]

    next_pose = copy.deepcopy(pose1)
    step_index = 1

    while step_index <= num_steps:
      next_pose.position.x += step_dist
      self.poses.poses.append(copy.deepcopy(next_pose))
      self.incrementalFlag.append(True)
      self.press_targets.append(press_amount)
      step_index += 1

press_schedule = PressSchedule()

def create_file():
  poses = np.zeros([len(press_schedule.poses.poses), 7])
  for index, pose in enumerate(press_schedule.poses.poses):
    poses[index, 0] = pose.position.x
    poses[index, 1] = pose.position.y
    poses[index, 2] = pose.position.z
    poses[index, 3] = pose.orientation.x
    poses[index, 4] = pose.orientation.y
    poses[index, 5] = pose.orientation.z
    poses[index, 6] = pose.orientation.w

  with h5py.File("pose_test.hdf5", "a") as f:
    f.create_dataset("poses", data=poses)

def read_file():
  with h5py.File('pose_test.hdf5', 'r') as f:
    dset = f['poses']
    for data in dset:
      print(data)

def main():
  # create_file()
  read_file()

if __name__ == "__main__":
  main()