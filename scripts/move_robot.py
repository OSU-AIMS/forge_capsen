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

    self.home = Pose()
    self.home.position.x = 0.300
    self.home.position.y = 0.000
    self.home.position.z = 0.185
    self.home.orientation.x = 0
    self.home.orientation.y = 0.7071
    self.home.orientation.z = 0.0
    self.home.orientation.w = 0.7071

    self.furnace_approach = Pose()
    self.furnace_approach.position.x = -0.3363
    self.furnace_approach.position.y = -0.5798
    self.furnace_approach.position.z = 0.4000
    self.furnace_approach.orientation.x = 0.0019
    self.furnace_approach.orientation.y = -0.0052
    self.furnace_approach.orientation.z = 0.8660
    self.furnace_approach.orientation.w = -0.5000

    self.furnace_pick = Pose()
    self.furnace_pick.position.x = -0.4859
    self.furnace_pick.position.y = -0.8389
    self.furnace_pick.position.z = 0.4000
    self.furnace_pick.orientation.x = 0.0019
    self.furnace_pick.orientation.y = -0.0052
    self.furnace_pick.orientation.z = 0.8660
    self.furnace_pick.orientation.w = -0.5000

    xy_ratio = (self.furnace_approach.position.x - self.furnace_pick.position.x) / (self.furnace_approach.position.y - self.furnace_pick.position.y)

    self.pick_approach = copy.deepcopy(self.furnace_pick)
    self.pick_approach.position.y += 0.025
    self.pick_approach.position.x += (0.025 * xy_ratio)

    self.pick_lift = copy.deepcopy(self.furnace_pick)
    self.pick_lift.position.z += 0.03

    self.furnace_depart = copy.deepcopy(self.pick_lift)
    self.furnace_depart.position.y += 0.280
    self.furnace_depart.position.x += (0.280 * xy_ratio)

    # self.furnace_depart = Pose()
    # self.furnace_depart.position.x = -0.3234
    # self.furnace_depart.position.y = -0.5580
    # self.furnace_depart.position.z = 0.4500
    # self.furnace_depart.orientation.x = -0.0019
    # self.furnace_depart.orientation.y = 0.00052
    # self.furnace_depart.orientation.z = -0.8660
    # self.furnace_depart.orientation.w = 0.5000

    self.waypoint1 = Pose()
    self.waypoint1.position.x = 0.302
    self.waypoint1.position.y = -0.570
    self.waypoint1.position.z = 0.427
    self.waypoint1.orientation.x = -0.004
    self.waypoint1.orientation.y = 0.0004
    self.waypoint1.orientation.z = -0.515
    self.waypoint1.orientation.w = 0.857

    self.press_approach = Pose()
    self.press_approach.position.x = 0.801
    self.press_approach.position.y = -0.173
    self.press_approach.position.z = 0.200
    self.press_approach.orientation.x = 0.0
    self.press_approach.orientation.y = 0.0
    self.press_approach.orientation.z = 0.0
    self.press_approach.orientation.w = 1.0

    self.press_approach_far = copy.deepcopy(self.press_approach)
    self.press_approach_far.position.x -= 0.100


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

  def move_to_position_name(self, position_name, speed):
    """
    Parameters
    ----------
    speed : float
      A decimal value in [0,1] that commands the robot speed as a proportion of
      its max speed

    name: string
      must match a name specified in file action_locations.csv 
      supply arg in quotes
    """
    # Make a separate call to the service for each position in the list.
    response = self._move_to_joint_positions_client(
          type=1, name=position_name, abs_value=speed)

    if response.error:
      raise ValueError("failed to move to joint position: %s"%response.error)
      

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
  
  def get_current_pose(self):
    
    tf_listener = tf.TransformListener()
    pose = Pose()

    attempt = 1
    max_attempts = 5

    while attempt <= max_attempts:
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
        attempt += 1
        if attempt > 2:
          raise ValueError('Get_current_pose failed')

forging_client = ForgingClient()

def close_gripper():
    try:
        forging_client.set_io("GRIP", True) # close gripper
        time.sleep(1.5)
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
    
def open_gripper():
    try:
        forging_client.set_io("GRIP", False) # open gripper
        time.sleep(1.5)
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
  
def open_furnace():
  forging_client.set_io("DOOR", True)
  time.sleep(5)

def close_furnace():
  forging_client.set_io("DOOR", False)
  time.sleep(5)

def main():
  # Initialize
  rospy.init_node('forge_gp7_node')
  
  try:
    
    # open_furnace()
    # open_gripper()

    print(forging_client.get_current_pose())

    # forging_client.move_to_position_name("waypoint1", 0.03)

    # SCAN PART
    # forging_client.move_to_scanning_positions(0.03)

    # pose = forging_client.get_current_pose()
    # pose.position.x -= 0.06
    # forging_client.move_scanning_object_to_pose(pose, 0.03)

    # forging_client.move_to_position_name("press_approach", 0.03)

    # forging_client.move_scanning_object_to_pose(forging_client.waypoint1, .03)
    # forging_client.move_scanning_object_to_pose(forging_client.furnace_approach, .03)
    # forging_client.move_scanning_object_to_pose(forging_client.pick_approach, .03)
    # forging_client.move_scanning_object_to_pose(forging_client.furnace_pick, .03)
    # close_gripper()
    # forging_client.move_scanning_object_to_pose(forging_client.pick_lift, .03)
    # forging_client.move_scanning_object_to_pose(forging_client.furnace_depart, 0.03)
    # forging_client.move_scanning_object_to_pose(forging_client.waypoint1, .03)
    # close_furnace()

    # forging_client.move_to_position_name("press_approach", 0.03)
    # forging_client.move_scanning_object_to_pose(forging_client.press_approach_far, .03)
    # forging_client.move_scanning_object_to_pose(forging_client.press_approach, .03)
    
  except ValueError as err:
    print(err.args)
 

if __name__ == "__main__":
  main()