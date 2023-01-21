#!/usr/bin/env python
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: W. Hansen, M. Groeber, A. Buynak

# Description:
#
#

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
]


def close_gripper():
  try:
    forging_client.set_io("GRIP", True) # close gripper
    time.sleep(1)
  except Exception as e:
    print("service call faild: %s"%e)
    raise ValueError("open_gripper failed")
    
def open_gripper():
  try:
    forging_client.set_io("GRIP", False) # open gripper
    time.sleep(1)
  except Exception as e:
    print("service call failed: %s"%e)
    raise ValueError("open_gripper failed")
  
def open_furnace():
  try:
    forging_client.set_io("DOOR", True)
    time.sleep(5)
  except Exception as e:
    print("service call failed: %s"%e)
    raise ValueError("open_gripper failed")

def close_furnace():
  try:
    forging_client.set_io("DOOR", False)
    time.sleep(1)  
  except Exception as e:
    print("service call failed: %s"%e)
    raise ValueError("open_gripper failed")

def pose_from_incremental_move(start_pose, dx, dy, dz, drx, dry, drz):
  # expecting start_pose to be type geometry_msgs/Pose with point, quaternion representation
  # expecting dx, dy, dz to be incremental step in meters
  # expecting drx, dry, drz to be incremental rotations in degrees
  end_pose = Pose()
  end_pose.position.x = start_pose.position.x + dx
  end_pose.position.y = start_pose.position.y + dy
  end_pose.position.z = start_pose.position.z + dz
  # convert degrees to radians
  drx = drx*math.pi/180
  dry = dry*math.pi/180
  drz = drz*math.pi/180
  q_rot = quaternion_from_euler(drx, dry, drz, "sxyz")
  quaternion_start = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]
  end_pose.orientation = quaternion_multiply(q_rot, quaternion_start)
  return end_pose

class PressSchedule():

  def __init__(self):

    self.poses = PoseArray()
    self.hitMags = []
    self.minZs = []
    self.maxZs = []

  def read_hdf5_file(self):
    try:
      with h5py.File('/home/capsen/ws_forge/src/D3D_planning/IncrementalForming_HitSequence.hdf5', 'r') as f:

        keepPressing_dataset = f['keepPressing']
        self.keep_pressing = keepPressing_dataset[0]

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
        
        hitMag_dataset = f['hitMag']
        for data in hitMag_dataset:
          self.hitMags.append(copy.deepcopy(data))

        maxZ_dataset = f['maxZ']
        for data in maxZ_dataset:
          self.maxZs.append(copy.deepcopy(data))
        
        minZ_dataset = f['minZ']
        for data in minZ_dataset:
          self.minZs.append(copy.deepcopy(data))
    except Exception as err:
      print(err.args)
      raise ValueError("read_hdf5_file failed")

  def call_D3D(self):
    program_filepath = "/home/capsen/AIMS_D3D/DREAM3D-6.6.275.364368cf9-Linux-x86_64/bin/PipelineRunner"
    json_filepath = "/home/capsen/ws_forge/src/D3D_planning/IncrementalFormingPipeline.json"
    try:
      subprocess.call([program_filepath, "-p", json_filepath])
    except Exception as err:
      print(err.args)
      raise ValueError("call_D3D failed")

class Press():
  def __init__(self):
    # Enter IR temperature sensor calibration offset
    self.temperature_offset = 100

    # ROS subscribers
    self.position_sub = rospy.Subscriber('/coaliron/joint_states', JointState, self.position_callback)
    self.pss_sub = rospy.Subscriber('/coaliron/power_switch_state', Bool, self.pss_callback)
    self.temp_sub = rospy.Subscriber('/coaliron/temperature1', Temperature, self.temp_callback)
    self.tas_sub = rospy.Subscriber('/coaliron/temperature1_alarm1_state', Bool, self.tas_callback)
    self.fs_sub = rospy.Subscriber('/coaliron/fork_state', Bool, self.fs_callback)
    # self.move_active_sub = rospy.Subscriber('/coaliron/move_active', Bool, self.move_active_callback)

    # ROS publishers
    self.pub = rospy.Publisher('/coaliron/joint_command', Float64,queue_size=10)
    self.rate = rospy.Rate(5)

    self.press_top = 0.020

  def position_callback(self, data):
    self.position = data.position[0]

  def pss_callback(self, data):
    self.power_switch_state = data.data

  def temp_callback(self, data):
    self.raw_temperature = data.temperature
    self.adjusted_temperature = float(self.raw_temperature) + self.temperature_offset

  def tas_callback(self, data):
    self.temperature_alarm_state = data.data

  def fs_callback(self, data):
    self.fork_state = data.data

  def move_active_callback(self, data):
    self.move_active = data.data

  def halt_press(self):
    rospy.loginfo("Reached halt_press")
    try:
      stop_position = self.position
      self.publish_once(stop_position)
      return stop_position
    except ValueError as err:
      rospy.loginfo('Cannot halt press')
      raise

  def publish_once(self, press_target_pos):
    rospy.loginfo("Reached publish once; target position = {}".format(press_target_pos))
    if press_target_pos < 0:
      press_target_pos = 0
    connections = self.pub.get_num_connections()
    if connections > 0:
      self.pub.publish(press_target_pos)
      rospy.loginfo("Successfully published {}".format(press_target_pos))
    else:
      raise ValueError(r'No connection to /coaliron/joint_command')
         
  def press(self, target):
    # This function will block until press reaches position or timeout
    if self.power_switch_state == False:
      raise ValueError('Cannot execute press command with power switch off')

    start_time = time.time()
    timeout = 10

    if target > self.position:
      press_direction = "down"
    elif target < self.position:
      press_direction = "up"
    else:
      rospy.loginfo("Already at requested press position")
      return

    try:                               
      self.publish_once(target)
    except ValueError as err:
      raise ValueError('Failed to publish press target position', err.args)

    while (time.time() - start_time) < timeout:
      # this loop will block until press position reaches or overshoots target
      if press_direction == "down" and self.position > target:
        break
      elif press_direction == "up" and self.position < target:
        break
    
    if (time.time() - start_time) >= timeout:
      self.halt_press()
      raise ValueError('Press timeout before target reached', time.time(), start_time, timeout)
    else:
      rospy.loginfo('Successfully pressed. Current position = {}'.format(self.position))
    
  def press_in_steps(self, target):
    # This function breaks a single press target position into multiple steps, calling relieve forces after each step
    # Requires that the target position is lower (higher position value) than current position

    # step_dist is the amount that the press is moved down with each step
    step_dist = .001
    start_time = time.time()
    timeout = 30

    fx_tare = copy.deepcopy(ft_sensor.force.x)
    fy_tare = copy.deepcopy(ft_sensor.force.y)
    fz_tare = copy.deepcopy(ft_sensor.force.z)
    tx_tare = copy.deepcopy(ft_sensor.torque.x)
    ty_tare = copy.deepcopy(ft_sensor.torque.y)
    tz_tare = copy.deepcopy(ft_sensor.torque.z)

    if self.power_switch_state == False:
      raise RuntimeError("press_in_steps cannot execute press command with power switch off")

    try: 
      start_position = copy.deepcopy(self.position)
    except:
      raise RuntimeError('press_in_steps could not determine current press position')
    
    #initialize target to starting position
    step_target = start_position

    while (target > self.position) and (time.time() - start_time < timeout):
      if (target - self.position) > step_dist:
        while (step_target - self.position) < (0.5 * step_dist):
          step_target += step_dist
      else:
        step_target = target
      
      try:
        self.press(step_target)
      except ValueError as err:
        print(err.args)
        raise RuntimeError('press_in_steps failed to publish press target position')
      
      try: 
        forging_client.relieve_corrected_forces()
      except ValueError as err:
        self.halt_press()
        print(err.args)
        raise ValueError("press_in_steps failed call to relieve_relative_forces")
        
    if (time.time() - start_time) > timeout:
        self.halt_press()
        raise ValueError('Press timeout before target reached', time.time(), start_time, timeout)
    else:
      rospy.loginfo('Successfully pressed to %f', target)

  def press_incremental(self, increment):
    if self.position is None:
      raise ValueError("Cannot obtain press position for incremental move", increment)
    elif increment > 0:
      try:
        self.press(self.position + increment)
      except ValueError as err:
        print(err.args)
        raise ValueError("press_incremental failed call to press")
    elif increment < 0:
      try:
        self.press(self.position + increment)
      except ValueError as err:
        print(err.args)
        raise ValueError("press_incremental failed call to press")
    else:
      raise ValueError("press_incremental cannot execute zero distance move", increment)

  def seek_press_bottom(self):

    rospy.loginfo("Entered seek_press_bottom")
    
    if self.power_switch_state == False:
      raise ValueError('Cannot execute press command with power switch off')

    target_position = 0.200
    self.publish_once(target_position)
    timeout = 5
    position_threshold = .001
    start_time = time.time()

    while (time.time() - start_time) < timeout and target_position > self.position:
      previous_position = copy.deepcopy(self.position)
      time.sleep(0.5)
      if self.position - previous_position < position_threshold:
        press_position = self.halt_press()
        rospy.loginfo("Seek_press_bottom succeeded")
        return press_position
    
    press_position = self.halt_press()
    if self.position >= target_position:
      raise ValueError("seek_press_bottom failed: press reached target position")
    else:
      raise ValueError("seek_press_bottom failed: motion timeout")

class FTSensor:
  def __init__(self):
    # ROS subscribers
    self.ft_sub = rospy.Subscriber('/transformed_world', WrenchStamped, self.ft_callback)
    self.time = None

    # baseline describes the observed readings at room temp (29.4C at flange) holding workpiece
    # used to calculate corrected values when thermal changes occur
    self.baseline = WrenchStamped()
    self.baseline.wrench.force.x = 13.8
    self.baseline.wrench.force.y = 12.0
    self.baseline.wrench.force.z = -40.3
    self.baseline.wrench.torque.x = -1.1
    self.baseline.wrench.torque.y = 3.9
    self.baseline.wrench.torque.z = 0.1

    self.corrected = WrenchStamped()

    self.fx_shift = 0.0
    self.fy_shift = 0.0
    self.fz_shift = 0.0
    self.tx_shift = 0.0
    self.ty_shift = 0.0
    self.tz_shift = 0.0

  def ft_callback(self, data):
    self.wrench = data.wrench
    self.force = data.wrench.force
    self.torque = data.wrench.torque
    self.header = data.header
    self.time = data.header.stamp

    self.corrected.wrench.force.x = self.wrench.force.x - self.fx_shift
    self.corrected.wrench.force.y = self.wrench.force.y - self.fy_shift
    self.corrected.wrench.force.z = self.wrench.force.z - self.fz_shift
    self.corrected.wrench.torque.x = self.wrench.torque.x - self.tx_shift
    self.corrected.wrench.torque.y = self.wrench.torque.y - self.ty_shift
    self.corrected.wrench.torque.z = self.wrench.torque.z - self.tz_shift
    
  def set_shift(self, input_wrench):
    self.fx_shift = input_wrench.force.x - self.baseline.wrench.force.x
    self.fy_shift = input_wrench.force.y - self.baseline.wrench.force.y
    self.fz_shift = input_wrench.force.z - self.baseline.wrench.force.z
    self.tx_shift = input_wrench.torque.x - self.baseline.wrench.torque.x
    self.ty_shift = input_wrench.torque.y - self.baseline.wrench.torque.y
    self.tz_shift = input_wrench.torque.z - self.baseline.wrench.torque.z

  def is_publishing(self):
    # checks to see whether the subscriber callback has executed more recently than timeout value
    timeout = 5

    if self.time is not None:
      ros_time = rospy.get_rostime()
      msg_time = self.time
      elapsed_time = (ros_time.secs - msg_time.secs)
      if elapsed_time < timeout:
        return True
    else:
      return False

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
      raise ValueError("call to set_io failed")

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
      print("failed to move to joint position: %s"%response.error)
      raise ValueError("move_to_position_name failed")

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
      raise ValueError("move_to_scanning_positions failed")

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
      raise ValueError("move_scanning_object_to_pose failed")
  
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

  def seek_down(self):
    rospy.loginfo("starting seek_down")
    
    move_speed = 0.01

    if not ft_sensor.is_publishing():
      raise ValueError("seek_down determined that ft sensor not publishing")

    # df_threshold is change in force (N) that indicates successfully found bottom
    df_threshold = 5.0
    # max_dist is maximum distance (m) allowed to travel to find bottom
    max_dist = 0.050
    # target_dist will be incremented by dist_increment with each step down
    target_dist = 0.0
    dist_increment = .001
  
    start_force = copy.deepcopy(ft_sensor.force.z)

    try:
      seek_start_pose = copy.deepcopy(self.get_current_pose())
    except ValueError as err:
      print(err.args)
      raise ValueError("seek_down cannot determine seek_start_pose")

    print("start_force = {}".format(start_force))
    if (start_force > -2.0):
      rospy.logwarn("Seek_down starting force is higher than expected", ft_sensor.force.z)

    target_pose = copy.deepcopy(seek_start_pose)
    
    while abs(ft_sensor.force.z - start_force) < df_threshold and target_dist <= max_dist:
      target_dist += dist_increment
      print("z force = {}".format(ft_sensor.force.z))
      target_pose.position.z = seek_start_pose.position.z - target_dist
      try:
        forging_client.move_scanning_object_to_pose(target_pose, move_speed)
      except ValueError as err:
        print(err.args)
        raise ValueError("seek_down failed to move robot as expected")
    if (target_dist > max_dist):
      raise ValueError("seek_down failed; exceeded max seek distance", max_dist)
    else:
      rospy.loginfo("Seek_down succeeded. Z Force = {}".format(ft_sensor.force.z))

  def relieve_corrected_forces(self):
    rospy.loginfo("Reached relieve corrected forces")
    if not ft_sensor.is_publishing():
      raise ValueError("relieve_corrected_forces determined that ft sensor not publishing")

    # will attempt to adjust robot pose by dist_increment if between lower and upper threshold
    # will raise error if above upper threshold
    lower_force_threshold = 10
    upper_force_threshold = 60
    lower_torque_threshold = 1
    upper_torque_threshold = 6
    dist_increment = .0005
    move_speed = 0.01

    fx = ft_sensor.corrected.wrench.force.x
    fy = ft_sensor.corrected.wrench.force.y
    fz = ft_sensor.corrected.wrench.force.z
    tx = ft_sensor.corrected.wrench.torque.x
    ty = ft_sensor.corrected.wrench.torque.y
    tz = ft_sensor.corrected.wrench.torque.z

    rospy.loginfo("Start (corrected): fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))

    if (abs(fx) < lower_force_threshold) and (abs(fy) < lower_force_threshold) and (abs(fz) < lower_force_threshold) and (
      abs(tx) < upper_torque_threshold) and (abs(ty) < upper_torque_threshold) and (abs(tz) < upper_torque_threshold):
      rospy.loginfo("No forces to relieve")
      return

    # initialize target_pose (modified later)
    try:
      target_pose = copy.deepcopy(self.get_current_pose())
    except ValueError as err:
      print(err.args)
      raise ValueError("relieve_corrected_forces cannot determine current_pose")
    
    # initialize move_list and relief success flags to ensure we enter while loop
    move_list = [1]
    x_success = True
    y_success = True
    z_success = True

    # Continue checking and relieving forces until relieved or until relief moves fail to provide relief
    while 1 in move_list or -1 in move_list:

      if (abs(fx) > upper_force_threshold) or (abs(fy) > upper_force_threshold) or (
        abs(fz) > upper_force_threshold or abs(tx) > upper_torque_threshold) or (
        abs(ty) > upper_torque_threshold) or (abs(tz) > upper_torque_threshold):
          rospy.loginfo("Corrected force data:  fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))
          rospy.loginfo("Raw force data:  fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(ft_sensor.force.x, ft_sensor.force.y, ft_sensor.force.z, ft_sensor.torque.x, ft_sensor.torque.y, ft_sensor.torque.z))
          raise ValueError("relieve_corrected_forces error: exceeded upper force limit")

      # record starting forces and torques for later comparison
      fx_start = copy.deepcopy(fx)
      fy_start = copy.deepcopy(fy)
      fz_start = copy.deepcopy(fz)

      # determine relief directions
      # consider whether last attempt to relieve actually relieved forces
      if (fx < -lower_force_threshold) and x_success:
        x_dir = -1
      elif (fx > lower_force_threshold) and x_success:
        x_dir = 1
      else:
        x_dir = 0
      if (fy < -lower_force_threshold) and y_success:
        y_dir = -1
      elif (fy > lower_force_threshold) and y_success:
        y_dir = 1
      else:
        y_dir = 0
      if (fz < -lower_force_threshold) and z_success:
        z_dir = -1
      elif (fz > lower_force_threshold) and z_success:
        z_dir = 1
      else:
        z_dir = 0

      move_list = [x_dir, y_dir, z_dir]
      if 1 in move_list or -1 in move_list:
        
        target_pose.position.x += (x_dir*dist_increment)
        target_pose.position.y += (y_dir*dist_increment)
        target_pose.position.z += (z_dir*dist_increment)

        try:
          rospy.loginfo("moving to relieve forces. move_list = {}".format(move_list))
          rospy.loginfo("Target: x = {}, y = {}, z = {}".format(target_pose.position.x, target_pose.position.y, target_pose.position.z))
          self.move_scanning_object_to_pose(target_pose, move_speed)
        except ValueError as err:
          print(err.args)
          raise ValueError("relieve_corrected_forces failed to move scanning object to desired pose")
        
        # Check whether the move actually relieved force as intended
        # Reset flags to True if no motion was executed in corresponding direction
        fx = ft_sensor.corrected.wrench.force.x
        fy = ft_sensor.corrected.wrench.force.y
        fz = ft_sensor.corrected.wrench.force.z
        tx = ft_sensor.corrected.wrench.torque.x
        ty = ft_sensor.corrected.wrench.torque.y
        tz = ft_sensor.corrected.wrench.torque.z

        x_success_threshold = 10.0
        yz_success_threshold = 5.0

        if not x_dir  == 0:
          x_success = (abs(fx_start) - abs(fx)) > x_success_threshold
          
        if not y_dir  == 0:
          y_success = (abs(fy_start)- abs(fy)) > yz_success_threshold

        if not z_dir  == 0:
          z_success = (abs(fz_start)- abs(fz)) > yz_success_threshold

      else:
        if abs(fx) < lower_force_threshold and abs(fy) < lower_force_threshold and abs(fz) < lower_force_threshold:
          rospy.loginfo("Successfully relieved forces")
        else:
          rospy.loginfo("Forces remain but attempts to relieve were not successful")
          rospy.loginfo("Remaining (corrected): fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))

  def retrieve_part_from_furnace(self):
    move_speed = 0.03
    try:
      open_furnace()
      open_gripper()
      forging_client.move_scanning_object_to_pose(forging_client.waypoint1, move_speed)
      forging_client.move_scanning_object_to_pose(forging_client.furnace_approach, move_speed)
      forging_client.move_scanning_object_to_pose(forging_client.pick_approach, move_speed)
      forging_client.move_scanning_object_to_pose(forging_client.furnace_pick, move_speed)
      # time.sleep(1)
      close_gripper()
      forging_client.move_scanning_object_to_pose(forging_client.pick_lift, move_speed)
      forging_client.move_scanning_object_to_pose(forging_client.furnace_depart, move_speed)
      forging_client.move_scanning_object_to_pose(forging_client.waypoint1, move_speed)
      close_furnace()
      
    except Exception as err:
      print(err.args)
      raise ValueError("retrieve_part_from_furnace failed")

  def change_orientation(self, qx, qy, qz, qw):
    pose = self.get_current_pose()
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    forging_client.move_scanning_object_to_pose(pose, 0.03)

# Initialize
rospy.init_node('forge_gp7_node')

p = Press()
forging_client = ForgingClient()
ft_sensor = FTSensor()
press_schedule = PressSchedule()


rospy.loginfo("Waiting for press position")
got_press_position = False
while got_press_position == False:
  try:
    rospy.loginfo("Starting press position is {}".format(p.position))
  except:
    time.sleep(1)
  else:
    got_press_position = True

while ft_sensor.is_publishing() == False:
  print("Waiting for F-T sensor")
  time.sleep(1)


########
# Main #
########

def main():
  # service for moving to pre-recorded joint positions
  # type = 0: move to home position recorded in planner.yaml
  # type = 1: move to a position name which exists in action_locations.csv
  # type = 2: move to the series of scanning positions and generate the mesh.
  #           the scanning positions are saved as "in_hand_scanning_positions"
  #           in action_locations.csv

  rospy.wait_for_service('/planner/set_mode_service')

  # service for moving the scanned object to a pose relative to the robot
  rospy.wait_for_service('/planner/move_link_to_pose_service')

  # activate/deactivate DIO
  rospy.wait_for_service('/planner/set_io_service')

  # enable or disable collision checking for a specified link
  rospy.wait_for_service('/planner/set_collision_checking_service')


  # Move upper die to contact bottom die; return press position at bottom
  # try:
  #   press_bottom = p.seek_press_bottom()
  # except Exception as err:
  #   print(err.args)
  #   rospy.logerr("Failed to find press bottom")
  #   raise SystemExit

  # Move upper die back up by some amount
  # try:
  #   p.press(p.press_top)
  # except Exception as err:
  #   print(err.args)
  #   rospy.logerr("Failed to move press to top position")
  #   raise SystemExit

  # # RETRIEVE PART FROM FURNACE
  # try:
  #   forging_client.retrieve_part_from_furnace()
  # except ValueError as err:
  #   print(err.args)
  #   rospy.logerr("Failed to retrieve part from furnace")
  #   raise SystemExit

  # SCAN PART, CREATE OCCUPANCY MAP
  try:
    forging_client.move_to_scanning_positions(0.03)
  except Exception as err:
    print(err.args)
    rospy.logerr("Failed to scan part")
    raise SystemExit

  # CALL D3D, CREATE HDF5 FILE FROM OCCUPANCY MAP
  try:
    press_schedule.call_D3D()
  except Exception as err:
    print(err.args)
    rospy.logerr("Failed to call DREAM.3D")
    raise SystemExit

  # BUILD PRESS SCHEDULE FROM HDF5 FILE
  try:
    press_schedule.read_hdf5_file()
  except Exception as err:
    print(err.args)
    rospy.logerr("Failed to read HDF5 file")
    raise SystemExit

  while press_schedule.keep_pressing:

    press_schedule.keep_pressing = False

    for index, value in enumerate(zip(press_schedule.poses.poses, press_schedule.hitMags, press_schedule.minZs, press_schedule.maxZs)):
      pose = value[0]
      hitMag = value[1]
      minZ = value[2]
      maxZ = value[3]
      part_thickness = maxZ - minZ

      move_speed = 0.03
      press_entry_z_offset = .050
      seek_down_z_offset = 0.010

      press_start_offset = .010
      press_start = press_bottom - part_thickness - press_start_offset
      press_target = press_bottom - part_thickness + hitMag

      if index == 0:
        # only execute this for first pose in the list
        # move part to press_entry, which is first press location + entry z-offset
        entry_pose = copy.deepcopy(pose)
        entry_pose.position.z += press_entry_z_offset
        try:
          forging_client.move_to_position_name("waypoint1", move_speed)
          forging_client.move_scanning_object_to_pose(forging_client.press_approach_far, move_speed)
          forging_client.move_scanning_object_to_pose(forging_client.press_approach, move_speed)
        except Exception as err:
          print(err.args)
          rospy.logerr("Failed initial moves to press approach")
          raise SystemExit

        try:
          forging_client.move_scanning_object_to_pose(entry_pose, move_speed)
        except Exception as err:
          print(err.args)
          rospy.logerr("Failed to reach position target {} with value {}".format(index, value))
          raise SystemExit
      
      # move part to seek_down_start, which is first press location + seek_start z-offset
      seek_down_pose = copy.deepcopy(pose)
      seek_down_pose.position.z += seek_down_z_offset

      try:
        forging_client.move_scanning_object_to_pose(seek_down_pose, move_speed)
      except Exception as err:
        print(err.args)
        rospy.logerr("Failed to reach seek_down_pose of position target {} with value {}".format(index, value))
        raise SystemExit

      # set the force-torque shift value to compensate for temperature
      try:
        ft_sensor.set_shift(ft_sensor.wrench)
      except Exception as err:
        print(err.args)
        rospy.logerr("Failed to set FT sensor shift value")
        raise SystemExit

      # move part to contact lower die using force feedback
      try:
        forging_client.seek_down()
      except Exception as err:
        print(err.args)
        rospy.logerr("Failed to reach seek_down_pose of position target {} with value {}".format(index, value))
        raise SystemExit

      # move press to press_start, which is above part by amount press_start_offset
      try: 
        p.press(press_start)
      except Exception as err:
          print(err.args)
          rospy.logerr("Pressing to press_start failed")
          raise SystemExit

      # press part
      try: 
        p.press_in_steps(press_target)
      except ValueError as err:
        print(err.args)
      except RuntimeError as err:
        print(err.args)
        rospy.logerr("Press in steps failed")
        raise SystemExit

      # # move press back up
      try: 
        p.press(p.press_top)
      except Exception as err:
          print(err.args)
          rospy.logerr("Failed to move press to top position")
          raise SystemExit
      
      # raise part off of lower die before moving to next seek_down_start
      try:
        forging_client.move_incremental(0,0,seek_down_z_offset)
      except Exception as err:
        print(err.args)
        rospy.logerr("Failed to raise part off of lower die")
        raise SystemExit

    # leave press
    try:
      forging_client.move_incremental(0, 0, .05)
      forging_client.move_incremental(-.150, 0, 0)
      forging_client.move_scanning_object_to_pose(forging_client.press_approach_far, move_speed)
      forging_client.move_to_position_name("waypoint1", move_speed)
    except Exception as err:
      print(err.args)
      rospy.logerr("Failed moves to depart press")
      raise SystemExit
    
    # SCAN PART, CREATE OCCUPANCY MAP
    try:
      forging_client.move_to_scanning_positions(0.03)
    except Exception as err:
      print(err.args)
      rospy.logerr("Failed to scan part")
      raise SystemExit

    # CALL D3D, CREATE HDF5 FILE FROM OCCUPANCY MAP
    try:
      press_schedule.call_D3D()
    except Exception as err:
      print(err.args)
      rospy.logerr("Failed to call DREAM.3D")
      raise SystemExit

    # BUILD PRESS SCHEDULE FROM HDF5 FILE
    try:
      press_schedule.read_hdf5_file()
    except Exception as err:
      print(err.args)
      rospy.logerr("Failed to read HDF5 file")
      raise SystemExit

    # COMMENT THIS OUT TO ALLOW CONTINUOUS OPERATION
    press_schedule.keep_pressing = False

if __name__ == "__main__":
  main()