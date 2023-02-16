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

press_down_timeout = 7
press_up_timeout = 5
press_down_tolerance = .001
press_up_tolerance = .005
press_top = 0.0

def close_gripper():
    try:
        forging_client.set_io("GRIP", True) # close gripper
        time.sleep(1)
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
    
def open_gripper():
    try:
        forging_client.set_io("GRIP", False) # open gripper
        time.sleep(1)
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
  
def open_furnace():
  forging_client.set_io("DOOR", True)
  time.sleep(1)

def close_furnace():
  forging_client.set_io("DOOR", False)
  time.sleep(1)

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
    with h5py.File('IncrementalFormingFullRun.hdf5', 'r') as f:
      keepPressing_dataset = f['keepPressing']
      if keepPressing_dataset[0] == False:
        rospy.loginfo("Pressing complete!")
        raise SystemExit

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

  def call_D3D(self):
    program_filepath = "/home/capsen/AIMS_D3D/DREAM3D-6.6.332.3886a295b-Linux-x86_64/bin/PipelineRunner"
    json_filepath = "/home/capsen/Downloads/D3D/IncrementalFormingPipeline-LinuxTest.json"
    subprocess.call([program_filepath, "-p", json_filepath])

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
    if press_target_pos < 0:
      press_target_pos = 0
    connections = self.pub.get_num_connections()
    if connections > 0:
      self.pub.publish(press_target_pos)
      rospy.loginfo('Published %f', press_target_pos)
    else:
      raise ValueError(r'No connection to /coaliron/joint_command')
         
  def press(self, target, tolerance, timeout):
    # This function will block until press reaches position or timeout
    if self.power_switch_state == False:
      raise ValueError('Cannot execute press command with power switch off')

    start_time = time.time()

    try:                               
      self.publish_once(target)
    except ValueError as err:
      raise ValueError('Failed to publish press target position', err.args)

    while (time.time() - start_time) < timeout and abs(target - float(self.position)) > tolerance:
      # this loop will block until press position within tolerance
      pass
    
    if (time.time() - start_time) >= timeout:
      self.halt_press()
      raise ValueError('Press timeout before target reached', time.time(), start_time, timeout)
    else:
      rospy.loginfo('Successfully pre press_with_force_fb(self, target, tolerance, timeout)')
    # This function will publish a single target position and call relieve forces
    # Results are mixed: relieve forces does not keep up well with press
    if self.power_switch_state == False:
      raise ValueError("Cannot execute press command with power switch off")
    
    start_time = time.time()                      
    press_complete = False         
    
    # Must call publish_once() rather than press() because cannot be blocking call
    # otherwise, could not check FT sensor while moving press
    try:
      self.publish_once(target)
    except ValueError as err:
      print(err.args)
      raise ValueError('Failed to publish press target position')
    
    while (time.time() - start_time) < timeout and press_complete == False:
      if abs(target - float(self.position)) < tolerance:
        rospy.loginfo('Successfully pressed to %f', target)
        press_complete = True
      else:
        try: 
          forging_client.relieve_forces()
        except ValueError as err:
          self.halt_press()
          print(err.args)
          raise ValueError('Upper force limit detected before target reached', target, self.position)
      
    if (time.time() - start_time) > timeout:
      self.halt_press()
      # raise ValueError('Press timeout before target reached', time.time(), start_time, timeout)

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
      start_position = copy.deepcopy(self.position())
    except:
      raise RuntimeError('press_in_steps could not determine current press position')
    
    #initialize target to starting position
    step_target = start_position

    while (target > self.position()) and (time.time() - start_time < timeout):
      if (target - self.position()) > step_dist:
        step_target += step_dist
      else:
        step_target = target
      
      try:
        self.publish_once(step_target)
      except ValueError as err:
        print(err.args)
        raise RuntimeError('press_in_steps failed to publish press target position')
      
      while (step_target > self.position()) and (time.time() - start_time < timeout):
        try: 
          forging_client.relieve_relative_forces(fx_tare, fy_tare, fz_tare, tx_tare, ty_tare, tz_tare)
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
        self.press(self.position + increment, press_down_tolerance, press_down_timeout)
      except ValueError as err:
        print(err.args)
        raise ValueError("press_incremental failed call to press")
    elif increment < 0:
      try:
        self.press(self.position + increment, press_up_tolerance, press_up_timeout)
      except ValueError as err:
        print(err.args)
        raise ValueError("press_incremental failed call to press")
    else:
      raise ValueError("press_incremental cannot execute zero distance move", increment)

  def press_incremental_with_force_fb(self, increment):
    if self.position is None:
      raise ValueError("Cannot obtain press position for incremental move", increment)
    elif increment > 0:
      try:
        self.press_down_in_steps_with_force_fb(self.position + increment)
      except ValueError as err:
        print(err.args)
        raise ValueError('press_incremental_with_force_fb failed')
    else:
      raise ValueError("Cannot press up with force feedback. Requires positive, non-zero increment", increment)
 
  def seek_part_contact(self):

    rospy.loginfo("Entered seek_part_contact")
    
    if not ft_sensor.is_publishing():
      raise ValueError("seek_part_contact determined that ft sensor not publishing")

    # df_threshold is change in force (N) that indicates successfully found bottom
    df_threshold = 5.0
    dt_threshold = 1.0

    dist_increment = .005
    start_force = copy.deepcopy(ft_sensor.force.z)
    start_torque = copy.deepcopy(ft_sensor.torque.y)
    start_position = copy.deepcopy(self.position)
    target_position = start_position

    contact_made = False

    while not contact_made:
      at_target_position = abs(self.position - target_position) < press_down_tolerance
      if at_target_position:
        target_position += dist_increment
        self.publish_once(target_position)
      contact_made = abs(ft_sensor.force.z - start_force) >= df_threshold or abs(ft_sensor.torque.y - start_torque) >= dt_threshold
    
    press_position = self.halt_press()
    rospy.loginfo("Seek_part_contact succeeded")
    return press_position

  def seek_press_bottom(self):

    rospy.loginfo("Entered seek_press_bottom")

    dist_increment = .050
    timeout = 5
    start_position = copy.deepcopy(self.position)
    target_position = start_position

    bottom_found = False

    while not bottom_found:
      target_position += dist_increment
      start_time = time.time()
      self.publish_once(target_position)
      while (time.time() - start_time) < timeout and abs(target_position - float(self.position)) > press_down_tolerance:
        pass
      at_target_position = abs(self.position - target_position) < press_down_tolerance
      if not at_target_position:
        bottom_found = True
    
    press_position = self.halt_press()
    rospy.loginfo("Seek_press_bottom succeeded")
    return press_position

class FTSensor:
  def __init__(self):
    # ROS subscribers
    self.ft_sub = rospy.Subscriber('/transformed_world', WrenchStamped, self.ft_callback)

    self.max_allowed_torque = 10.0 # N-m, limited by GP7 T axis
    self.max_allowed_force = 69.0 # N, limited by GP7 max payload (7kg)
  
  def ft_callback(self, data):
    self.wrench = data.wrench
    self.force = data.wrench.force
    self.torque = data.wrench.torque
    self.header = data.header
    self.time = data.header.stamp

  def is_publishing(self):
    # checks to see whether the subscriber callback has executed more recently than timeout value
    timeout = 0.1
    if rospy.get_time() - self.time < timeout:
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

  def relieve_forces(self):
    rospy.loginfo("Starting to relieve forces")

    move_speed = .01

    if not ft_sensor.is_publishing():
      raise ValueError("relieve_forces determined that ft sensor not publishing")

    fx = ft_sensor.force.x
    fy = ft_sensor.force.y
    fz = ft_sensor.force.z
    tx = ft_sensor.torque.x
    ty = ft_sensor.torque.y
    tz = ft_sensor.torque.z
    rospy.loginfo("fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))

    lower_force_threshold = 30
    upper_force_threshold = 80
    upper_torque_threshold = 10

    dist_increment = .0005
    x_dir = 1
    y_dir = 1
    z_dir = 1
    
    while (abs(fx) > lower_force_threshold) or (abs(fy) > lower_force_threshold) or (abs(fz) > lower_force_threshold):
      if (fx < -lower_force_threshold):
        x_dir = -1
      elif (fx > lower_force_threshold):
        x_dir = 1
      else:
        x_dir = 0
      if (fy < -lower_force_threshold):
        y_dir = -1
      elif (fy > lower_force_threshold):
        y_dir = 1
      else:
        y_dir = 0
      if (fz < -lower_force_threshold):
        z_dir = -1
      elif (fz > lower_force_threshold):
        z_dir = 1
      else:
        z_dir = 0

      try:
        target_pose = copy.deepcopy(self.get_current_pose())
      except ValueError as err:
        print(err.args)
        raise ValueError("Cannot determine current_pose to adjust to relieve forces")

      target_pose.position.x += (x_dir*dist_increment)
      target_pose.position.y += (y_dir*dist_increment)
      target_pose.position.z += (z_dir*dist_increment)

      try:
        self.move_scanning_object_to_pose(target_pose, move_speed)
      except ValueError as err:
        print(err.args)
        raise ValueError("relieve_forces failed to move scanning object to desired pose")

      fx = ft_sensor.force.x
      fy = ft_sensor.force.y
      fz = ft_sensor.force.z
      tx = ft_sensor.torque.x
      ty = ft_sensor.torque.y
      tz = ft_sensor.torque.z

      if (abs(fx) > upper_force_threshold) or (abs(fy) > upper_force_threshold) or (abs(fz) > upper_force_threshold or abs(tx) > upper_torque_threshold) or (abs(ty) > upper_torque_threshold) or (abs(tz) > upper_torque_threshold):
        rospy.loginfo("fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))
        raise ValueError("relieve_forces failed; Exceeded upper force limit")    
    
    rospy.loginfo("relieve_forces succeeded")

  def relieve_relative_forces(self, fx_tare, fy_tare, fz_tare, tx_tare, ty_tare, tz_tare):
    rospy.loginfo("Reached relieve relative forces")
    if not ft_sensor.is_publishing():
      raise ValueError("relieve_relative_forces determined that ft sensor not publishing")

    move_speed = 0.01

    fx = ft_sensor.force.x
    fy = ft_sensor.force.y
    fz = ft_sensor.force.z
    tx = ft_sensor.torque.x
    ty = ft_sensor.torque.y
    tz = ft_sensor.torque.z

    fx_net = fx - fx_tare
    fy_net = fy - fy_tare
    fz_net = fz - fz_tare
    tx_net = tx - tx_tare
    ty_net = ty - ty_tare
    tz_net = tz - tz_tare

    rospy.loginfo("Start: fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))

    # will attempt to adjust robot pose by dist_increment if between lower and upper threshold
    # will raise error if above upper threshold
    lower_force_threshold = 10
    upper_force_threshold = 80
    lower_torque_threshold = 1
    upper_torque_threshold = 8
    dist_increment = .0005

    # Check upper thresholds and raise error if exceeded
    if (abs(fx_net) > upper_force_threshold) or (abs(fy_net) > upper_force_threshold) or (
      abs(fz_net) > upper_force_threshold or abs(tx_net) > upper_torque_threshold) or (
      abs(ty_net) > upper_torque_threshold) or (abs(tz_net) > upper_torque_threshold):
        rospy.loginfo("fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))
        raise ValueError("relieve_relative_forces error: exceeded upper force limit") 

    # initialize target_pose (modified later)
    try:
      target_pose = copy.deepcopy(self.get_current_pose())
    except ValueError as err:
      print(err.args)
      raise ValueError("relieve_relative_forces cannot determine current_pose")
    
    # initialize move_list and relief success flags to ensure we enter while loop
    move_list = [1]
    x_neg_success = True
    x_pos_success = True
    y_neg_success = True
    y_pos_success = True
    z_neg_success = True
    z_pos_success = True

    # Continue checking and relieving forces until relieved or until relief moves fail to provide relief
    while 1 in move_list or -1 in move_list:

      # record starting forces and torques for later comparison
      fx_start = copy.deepcopy(fx)
      fy_start = copy.deepcopy(fy)
      fz_start = copy.deepcopy(fz)
      tx_start = copy.deepcopy(tx)
      ty_start = copy.deepcopy(ty)
      tz_start = copy.deepcopy(tz)

      # determine relief directions
      # consider whether last attempt to relieve actually relieved forces
      if (fx_net < -lower_force_threshold) and x_neg_success:
        x_dir = -1
      elif (fx_net > lower_force_threshold) and x_pos_success:
        x_dir = 1
      else:
        x_dir = 0
      if (fy_net < -lower_force_threshold) and y_neg_success:
        y_dir = -1
      elif (fy_net > lower_force_threshold) and y_pos_success:
        y_dir = 1
      else:
        y_dir = 0
      if (fz_net < -lower_force_threshold) and z_neg_success:
        z_dir = -1
      elif (fz_net > lower_force_threshold) and z_pos_success:
        z_dir = 1
      else:
        z_dir = 0

      move_list = [x_dir, y_dir, z_dir]
      if 1 in move_list or -1 in move_list:
        
        target_pose.position.x += (x_dir*dist_increment)
        target_pose.position.y += (y_dir*dist_increment)
        target_pose.position.z += (z_dir*dist_increment)

        try:
          self.move_scanning_object_to_pose(target_pose, move_speed)
        except ValueError as err:
          print(err.args)
          raise ValueError("relieve_relative_forces failed to move scanning object to desired pose")
        
        # Check whether the move actually relieved force as intended
        # Reset flags to True if no motion was executed in corresponding direction
        if x_dir == -1:
          x_neg_success = (fx_net > fx_start)
          x_pos_success = True
        elif x_dir == 1:
          x_pos_success = (fx_net < fx_start)
          x_neg_success = True
        else:
          x_neg_success = True
          x_pos_success = True

        if y_dir == -1:
          y_neg_success = (fy_net > fy_start)
          y_pos_success = True
        elif y_dir == 1:
          y_pos_success = (fy_net < fy_start)
          y_neg_success = True
        else:
          y_neg_success = True
          y_pos_success = True

        if z_dir == -1:
          z_neg_success = (fz_net > fz_start)
          z_pos_success = True
        elif z_dir == 1:
          z_pos_success = (fz_net < fz_start)
          z_neg_success = True
        else:
          z_neg_success = True
          z_pos_success = True

        success_flags = [x_neg_success, x_pos_success, y_neg_success, y_pos_success, z_neg_success, z_pos_success]

      else:
        if False in success_flags:
          rospy.loginfo("Last attempt to relieve a force failed {}".format(success_flags))
        else:
          # no forces present to be relieved
          pass

  def retrieve_part_from_furnace(self):
    pick = Pose()
    pick.position.x = 0.728
    pick.position.y = -0.207
    pick.position.z = 0.204
    pick.orientation.x = 0.0
    pick.orientation.y = 0.0
    pick.orientation.z = 0.0
    pick.orientation.w = 1.0
    
    furnace_approach = copy.deepcopy(pick)
    furnace_approach.position.y += 0.2
    furnace_approach.position.x += 0.2

    pick_approach = copy.deepcopy(pick)
    pick_approach.position.x += 0.030
    pick_approach.position.y += 0.030

    pick_lift = copy.deepcopy(pick)
    pick_lift.position.z += 0.020

    pick_depart = copy.deepcopy(pick_lift)
    pick_depart.position.x += 0.05

    waypoint1 = Pose()
    waypoint1.position.x = 0.801
    waypoint1.position.y = -0.173
    waypoint1.position.z = 0.200
    waypoint1.orientation.x = 0.0
    waypoint1.orientation.y = 0.0
    waypoint1.orientation.z = 0.0
    waypoint1.orientation.w = 1.0

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

  do_presssing = False

  if do_presssing:
 

    # Move upper die to contact bottom die; return press position at bottom
    press_bottom = p.seek_press_bottom()
    # Move upper die back up by some amount
    p.press(press_top, press_up_tolerance, press_up_timeout)

    # RETRIEVE PART FROM FURNACE
    # SCAN PART
    # CALL D3D

    press_schedule.read_hdf5_file()

    while press_schedule.keep_pressing:

      for index, value in enumerate(zip(press_schedule.poses.poses, press_schedule.press_targets, press_schedule.minZs, press_schedule.maxZs)):
        pose = value[0]
        hitMag = value[1]
        minZ = value[2]
        maxZ = value[3]
        part_thickness = maxZ - minZ

        move_speed = 0.03
        press_entry_z_offset = .050
        seek_down_z_offset = 0.010

        press_start_offset = .010
        press_start = press_bottom + part_thickness + press_start_offset
        press_target = press_bottom + part_thickness - hitMag

        if index == 0:
          # only execute this for first pose in the list
          # move part to press_entry, which is first press location + entry z-offset
          entry_pose = copy.deepcopy(pose)
          entry_pose.position.z += press_entry_z_offset
          try:
            forging_client.move_scanning_object_to_pose(entry_pose, move_speed)
          except ValueError as err:
            print(err.args)
            rospy.logerr("Failed to reach position target {} with value {}".format(index, value))
            raise SystemExit
        
        # move part to seek_down_start, which is first press location + seek_start z-offset
        seek_down_pose = copy.deepcopy(pose)
        seek_down_pose.position.z += seek_down_z_offset

        try:
          forging_client.move_scanning_object_to_pose(seek_down_pose, move_speed)
        except ValueError as err:
          print(err.args)
          rospy.logerr("Failed to reach seek_down_pose of position target {} with value {}".format(index, value))
          raise SystemExit

        # move part to contact lower die using force feedback
        try:
          forging_client.seek_down()
        except ValueError as err:
          print(err.args)
          rospy.logerr("Failed to reach seek_down_pose of position target {} with value {}".format(index, value))
          raise SystemExit

        # move press to press_start, which is above part by amount press_start_offset
        try: 
          p.press(press_start, press_down_tolerance, press_down_timeout)
        except ValueError as err:
            print(err.args)
            raise SystemExit

        # press part
        try: 
          p.press_in_steps(press_target)
        except ValueError as err:
            print(err.args)
        except RuntimeError as err:
          print(err.args)
          raise SystemExit

        # move press back up
        try: 
          p.press(press_top)
        except ValueError as err:
            print(err.args)
            raise SystemExit
        
        # raise part off of lower die before moving to next seek_down_start
        try:
          forging_client.move_incremental(0,0,0.02)
        except ValueError as err:
          print(err.args)
          rospy.logerr("Failed to raise part off of lower die")
          raise SystemExit

      # SCAN PART
      # CALL D3D
      press_schedule.read_hdf5_file()


  else:
    # forging_client.move_scanning_object_to_pose(waypoint1, move_speed)
    pass

if __name__ == "__main__":
  main()