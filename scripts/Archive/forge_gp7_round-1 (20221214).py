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
# import rospkg
from geometry_msgs.msg import Pose, Point, PoseArray, WrenchStamped, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Temperature
from capsen_vision.srv import SetProperty, MoveLinkToPose
# from tf_conversions import transformations
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

press_down_timeout = 7
press_up_timeout = 5
press_down_tolerance = .0004
press_up_tolerance = .005

def gripper_close():
    try:
        forging_client.set_gripper_state(True) # close gripper
        time.sleep(1)
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
    
def gripper_open():
    try:
        forging_client.set_gripper_state(False) # open gripper
        time.sleep(1)
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
  

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
    # self.move_active_sub = rospy.Subscriber('/coaliron/mov_active', Bool, self.move_active_callback)

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

  def halt_press(self):
    rospy.loginfo("Reached halt_press")
    try:
      self.publish_once(self.position)
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
    if self.power_switch_state == False:
      raise ValueError('Cannot execute press command with power switch off')
    if abs(target - float(self.position)) < tolerance:
      rospy.loginfo('Already at requested location. Target=%f, Position=%f, Tolerance=%f', target, self.position, tolerance)
      raise

    start_time = time.time()

    try:                               
      self.publish_once(target)
    except ValueError as err:
      raise ValueError('Failed to publish press target position', err.args)

    while (time.time() - start_time) < timeout and abs(target - float(self.position)) > tolerance:
      time.sleep(0.1)
    
    if (time.time() - start_time) >= timeout:
      self.halt_press()
      raise ValueError('Press timeout before target reached', time.time(), start_time, timeout)
    else:
      rospy.loginfo('Successfully pressed to %f', target)

  def press_with_force_fb(self, target, tolerance, timeout, effort):
    if self.power_switch_state == False:
      raise ValueError("Cannot execute press command with power switch off")
    if abs(target - float(self.position)) < tolerance:
      raise ValueError('Already at requested location', target, self.position, tolerance)
    
    start_time = time.time()                      
    press_complete = False         
    
    # Must call publish_once() rather than press() because cannot be blocking call
    # otherwise, could not check FT sensor while moving press
    try:
      self.publish_once(target)
    except ValueError as err:
      raise ValueError('Failed to publish press target position', err.args)
    
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

  def press_incremental(self, increment):
    if self.position is None:
      raise ValueError("Cannot obtain press position for incremental move", increment)
    elif increment > 0:
      return self.press(self.position + increment, press_down_tolerance, press_down_timeout)
    elif increment < 0:
      return self.press(self.position + increment, press_up_tolerance, press_up_timeout)
    else:
      raise ValueError("Cannot execute zero distance move", increment)

  def press_incremental_with_force_fb(self, increment, effort):
    if self.position is None:
      raise ValueError("Cannot obtain press position for incremental move", increment)
    elif increment > 0:
      # using non-standard press_down_timeout because calls to relieve_forces take more time
      try:
        self.press_with_force_fb(self.position + increment, press_down_tolerance, 30, effort)
      except ValueError as err:
        print(err.args)
        raise ValueError('press_incremental_with_force_fb failed')
    elif increment < 0:
      try:
        self.press_with_force_fb(self.position + increment, press_up_tolerance, press_up_timeout, effort)
      except ValueError as err:
        print(err.args)
        raise ValueError('press_incremental_with_force_fb failed')
    else:
      raise ValueError("Cannot execute zero distance move", increment)

  def old_seek_part_contact(self):

    rospy.loginfo("Entered seek_part_contact")

    # df_threshold is change in force (N) that indicates successfully found bottom
    df_threshold = 10.0
    # max_dist is maximum distance (m) allowed to travel to find bottom
    max_dist = 0.10
    # total_dist incremented by dist_increment with each step
    total_dist = 0.0
    dist_increment = .002
    start_force = copy.deepcopy(ft_sensor.force.z)

    while (total_dist <= max_dist) and abs(ft_sensor.force.z - start_force) < df_threshold: 
      p.press_incremental(dist_increment)
      time.sleep(0.1)
    
    if total_dist > max_dist:
      raise ValueError("Seek_part_contact failed: Exceeded max_dist", max_dist)
    else:
      rospy.loginfo("Seek_part_contact succeeded")
  
  def seek_part_contact(self):

    rospy.loginfo("Entered seek_part_contact")

    # df_threshold is change in force (N) that indicates successfully found bottom
    df_threshold = 10.0
    # max_dist is maximum distance (m) allowed to travel to find bottom
    max_dist = 0.10
    # total_dist incremented by dist_increment with each step
    total_dist = 0.0
    dist_increment = .002
    start_force = copy.deepcopy(ft_sensor.force.z)

    while (total_dist <= max_dist) and abs(ft_sensor.force.z - start_force) < df_threshold: 
      p.press_incremental(dist_increment)
      time.sleep(0.1)
    
    if total_dist > max_dist:
      raise ValueError("Seek_part_contact failed: Exceeded max_dist", max_dist)
    else:
      rospy.loginfo("Seek_part_contact succeeded")

  def move_active_callback(self, data):
    self.move_active = data.data


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

  def ft_limit_check(self, effort):
    # effort is the percentage of max allowed to be evaluated
    # initialize alarm to false
    alarm = False

    # set alarm True if any force or torque exceeds limit
    if abs(self.force.x) > (self.max_allowed_force * effort):
      alarm = True
      rospy.loginfo("Exceeded x-axis force limit")
    if abs(self.force.y) > (self.max_allowed_force * effort):
      alarm = True
      rospy.loginfo("Exceeded y-axis force limit")
    if abs(self.force.z) > (self.max_allowed_force * effort):
      alarm = True
      rospy.loginfo("Exceeded z-axis force limit")
    if abs(self.torque.x) > (self.max_allowed_torque * effort):
      alarm = True
      rospy.loginfo("Exceeded x-axis torque limit")
    if abs(self.torque.y) > (self.max_allowed_torque * effort):
      alarm = True
      rospy.loginfo("Exceeded y-axis torque limit")
    if abs(self.torque.z) > (self.max_allowed_torque * effort):
      alarm = True
      rospy.loginfo("Exceeded z-axis torque limit")
    return alarm


class ForgingClient:
  def move_home(self):
    response = self._move_to_joint_positions_client(type=0)

    if response.error:
      print("failed to move home: %s"%response.error)

  def move_incremental(self, dx, dy, dz):
    target_pose = self.get_current_pose()
    if target_pose == 1:
      rospy.logerr("could not obtain current pose to generate incremental move")
      return 1
    else:
      target_pose.position.x += dx
      target_pose.position.y += dy
      target_pose.position.z += dz
      self.move_scanning_object_to_pose(target_pose)

  def move_to_position_names(self):
    # Make a separate call to the service for each position in the list.
    # The service will block this thread until it either fails or finishes
    # moving to the desired location.
    for position_name in POSITION_NAMES:
      response = self._move_to_joint_positions_client(
          type=1, name=position_name)

  def move_to_scanning_positions(self):
    response = self._move_to_joint_positions_client(type=2)

    if response.error:
      print("failed to move home: %s"%response.error)

  def move_scanning_object_to_pose(self, pose):
    response = self._move_link_to_pose_client(link="object_link", pose=pose)

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
      except:
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

  def __init__(self):
    self._move_to_joint_positions_client = rospy.ServiceProxy(
        '/planner/set_mode_service', SetProperty)

    self._move_link_to_pose_client = rospy.ServiceProxy(
        '/planner/move_link_to_pose_service', MoveLinkToPose)

    self._set_gripper_state_client = rospy.ServiceProxy(
        '/planner/set_gripper_state_service', SetProperty)

  def set_gripper_state(self, close):
      response = self._set_gripper_state_client(on_off=close)

      if response.error:
        print("failed to set gripper state: %s"%response.error)
  
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

  def pose_from_incremental_move(self, start_pose, dx, dy, dz, drx, dry, drz):
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
    q_rot = quaternion_from_euler(drx, dry, drz)
    test_rot = quaternion_from_euler(1, 0, 0)
    end_pose.orientation = quaternion_multiply(q_rot, test_rot)
    # end_pose.orientation = quaternion_multiply(q_rot, start_pose.orientation)
    return end_pose

  def seek_down(self):
    rospy.loginfo("starting seek_down")

    # min_start_force must be present to begin seeking; this can probably be removed
    # or improved. Acts as check that FT sensor is operating properly. Units = N
    max_start_force = -2.0

    # df_threshold is change in force (N) that indicates successfully found bottom
    df_threshold = 5.0
    # max_dist is maximum distance (m) allowed to travel to find bottom
    max_dist = 0.050
    # target_dist will be incremented by dist_increment with each step down
    target_dist = 0.0
    dist_increment = .001
    
    # TODO: check that ft_sensor is publishing
    start_force = copy.deepcopy(ft_sensor.force.z)

    try:
      seek_start_pose = copy.deepcopy(self.get_current_pose())
    except ValueError as err:
      raise ValueError("Cannot determine seek_start_pose", err.arg)

    print("start_force = {}".format(start_force))

    target_pose = copy.deepcopy(seek_start_pose)
    
    if (start_force > max_start_force):
      raise ValueError("Seek_down failed; Starting force is higher than expected", ft_sensor.force.z)
    while abs(ft_sensor.force.z - start_force) < df_threshold and target_dist <= max_dist:
      target_dist += dist_increment
      print("z force = {}".format(ft_sensor.force.z))
      target_pose.position.z = seek_start_pose.position.z - target_dist
      forging_client.move_scanning_object_to_pose(target_pose)
      time.sleep(.1)
    if (target_dist > max_dist):
      raise ValueError("seek_down failed; exceeded max seek distance", max_dist)
    else:
      rospy.loginfo("Seek_down succeeded. Z Force = {}".format(ft_sensor.force.z))

  def relieve_forces(self):
    rospy.loginfo("Starting to relieve forces")
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
        raise ValueError("Cannot determine current_pose to adjust to relieve forces", err.arg)

      target_pose.position.x += (x_dir*dist_increment)
      target_pose.position.y += (y_dir*dist_increment)
      target_pose.position.z += (z_dir*dist_increment)

      try:
        self.move_scanning_object_to_pose(target_pose)
      except:
        raise ValueError("Failed to move scanning object to desired pose")

      fx = ft_sensor.force.x
      fy = ft_sensor.force.y
      fz = ft_sensor.force.z
      tx = ft_sensor.torque.x
      ty = ft_sensor.torque.y
      tz = ft_sensor.torque.z

      if (abs(fx) > upper_force_threshold) or (abs(fy) > upper_force_threshold) or (abs(fz) > upper_force_threshold or abs(tx) > upper_torque_threshold) or (abs(ty) > upper_torque_threshold) or (abs(tz) > upper_torque_threshold):
        rospy.loginfo("fx = {}, fy = {}, fz = {}, tx = {}, ty = {}, tz = {}".format(fx, fy, fz, tx, ty, tz))
        raise ValueError("Relieve_force failed; Exceeded upper force limit")    
    
    rospy.loginfo("Relieve_force succeeded")

# Initialize
rospy.init_node('forge_gp7_node')

p = Press()
forging_client = ForgingClient()
ft_sensor = FTSensor()

rospy.loginfo("Waiting for subscribers to initialize")
time.sleep(5)

try:
  rospy.loginfo("Starting press position is {}".format(p.position))
except:
  rospy.loginfo("Unable to read press position. Quitting program.")
  raise SystemExit

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

  # set gripper state (closed or open)
  rospy.wait_for_service('/planner/set_gripper_state_service')

  # while not rospy.is_shutdown():
  #   # rospy.loginfo(ft_sensor.wrench)    # pose_next = forging_client.get_current_pose()
    # pose_next.position.z -= .001
    # pose_next.position.x -= .0005
  #   rospy.logerr("Excessive force / torque detected: {}".format(ft_safety_response))
  
  # First press position of object_link:   
  press_start_pose = Pose()

  # Desired press start that we can't yet reach:
  # press_start_pose.position.x = 0.913
  # press_start_pose.position.y = -0.135 
  # press_start_pose.posiSystemExittion.z = 0.190
  # press_start_pose.orientation.x = -0.383
  # press_start_pose.orientation.y = 0.001
  # press_start_pose.orientation.z = -0.002
  # press_start_pose.orientation.w = 0.924

  # Substitute start pose:
  press_start_pose.position.x = 0.910
  press_start_pose.position.y = -0.135 
  press_start_pose.position.z = 0.188
  press_start_pose.orientation.x = -0.383
  press_start_pose.orientation.y = -0.011
  press_start_pose.orientation.z = -0.004
  press_start_pose.orientation.w = 0.924

  waypoint1 = Pose()
  waypoint1.position.x = 0.366
  waypoint1.position.y = -0.582
  waypoint1.position.z = 0.366
  waypoint1.orientation.x = 0.890
  waypoint1.orientation.y = -0.457
  waypoint1.orientation.z = 0
  waypoint1.orientation.w = 0.924

  seek_down_start_a = Pose()
  seek_down_start_a.position.x = 0.894
  seek_down_start_a.position.y = -0.135
  seek_down_start_a.position.z = 0.206
  seek_down_start_a.orientation.x = -0.381
  seek_down_start_a.orientation.y = -0.005
  seek_down_start_a.orientation.z = -0.002
  seek_down_start_a.orientation.w = 0.924

  seek_down_start_b = Pose()
  seek_down_start_b.position.x = 0.894
  seek_down_start_b.position.y = -0.135
  seek_down_start_b.position.z = 0.206
  seek_down_start_b.orientation.x = 0.932
  seek_down_start_b.orientation.y = 0.002
  seek_down_start_b.orientation.z = 0.005
  seek_down_start_b.orientation.w = -0.363

  seek_down_start_c = Pose()
  seek_down_start_c.position.x = 0.894
  seek_down_start_c.position.y = -0.135
  seek_down_start_c.position.z = 0.206
  seek_down_start_c.orientation.x = -0.706
  seek_down_start_c.orientation.y = -0.004
  seek_down_start_c.orientation.z = -0.004
  seek_down_start_c.orientation.w = 0.708

  seek_down_start_d = Pose()
  seek_down_start_d.position.x = 0.894
  seek_down_start_d.position.y = -0.135
  seek_down_start_d.position.z = 0.206
  seek_down_start_d.orientation.x = 1.000
  seek_down_start_d.orientation.y = 0
  seek_down_start_d.orientation.z = 0.005
  seek_down_start_d.orientation.w = 0.006

  # position e is octagon corner 1
  seek_down_start_e = Pose()
  seek_down_start_e.position.x = 0.897
  seek_down_start_e.position.y = -0.135
  seek_down_start_e.position.z = 0.206
  seek_down_start_e.orientation.x = -0.533
  seek_down_start_e.orientation.y = 0
  seek_down_start_e.orientation.z = 0
  seek_down_start_e.orientation.w = 0.846

  # position f is octagon corner 2
  seek_down_start_f = Pose()
  seek_down_start_f.position.x = 0.897
  seek_down_start_f.position.y = -0.135
  seek_down_start_f.position.z = 0.206
  seek_down_start_f.orientation.x = -0.288
  seek_down_start_f.orientation.y = 0
  seek_down_start_f.orientation.z = 0
  seek_down_start_f.orientation.w = 0.974

  # position g is octagon corner 3
  seek_down_start_g = Pose()
  seek_down_start_g.position.x = 0.909
  seek_down_start_g.position.y = -0.135
  seek_down_start_g.position.z = 0.206
  seek_down_start_g.orientation.x = 0.194
  seek_down_start_g.orientation.y = 0
  seek_down_start_g.orientation.z = 0
  seek_down_start_g.orientation.w = 0.981

  # position h is octagon corner 4
  seek_down_start_h = Pose()
  seek_down_start_h.position.x = 0.909
  seek_down_start_h.position.y = -0.135
  seek_down_start_h.position.z = 0.206
  seek_down_start_h.orientation.x = 0.517
  seek_down_start_h.orientation.y = 0
  seek_down_start_h.orientation.z = 0
  seek_down_start_h.orientation.w = 0.856

  try:

    # keepGoing = True
    # while keepGoing:
    #   try:
    #     forging_client.relieve_forces()
    #   except ValueError as err:
    #     print(err)
    #     print("Test")
    #     raise

    # forging_client.move_scanning_object_to_pose(waypoint1)
    # forging_client.move_scanning_object_to_pose(press_approach)
    
    # initial press move if necessary; if not, comment out or remove
    # try: 
    #   p.press_incremental(-.05)
    # except ValueError as err:
    #   print(err.args)
    #   raise SystemExit

    try:
      forging_client.move_incremental(0,0,0.02)
    except:
      rospy.logerr("Failed initial move")
      raise SystemExit

    # press side a

    try:
      forging_client.move_scanning_object_to_pose(seek_down_start_a)
    except ValueError as err:
      print(err.args)
      rospy.logerr("Failed to reach seek_down_start_a")
      raise SystemExit
    else:
      try:
        forging_client.seek_down()
      except ValueError as err:
        print(err.args)
        raise SystemExit
      else:
        try:
          p.seek_part_contact()
        except ValueError as err:
          print(err.args)
          raise SystemExit
        else:
          try:
            # args are (distance, effort)
            p.press_incremental_with_force_fb(.002, 0.6)
          except ValueError as err:
            print(err.args)
            raise SystemExit
          else:
            rospy.loginfo("Pressing at position A succeeded")

    # move press back up and move up off of bottom die
    try: 
      p.press_incremental(-.07)
    except ValueError as err:
        print(err.args)
        raise SystemExit
      
    try:
      forging_client.move_incremental(0,0,0.02)
    except:
      rospy.logerr("Failed to raise part off of lower die")
      raise SystemExit

    # press side b
    try:
      something = forging_client.move_scanning_object_to_pose(seek_down_start_b)
    except ValueError as err:
      rospy.logerr("Failed to reach seek_down_start_b")
      raise SystemExit
    else:
      try:
        forging_client.seek_down()
      except ValueError as err:
        print(err.args)
        raise SystemExit
      else:
        try:
          p.seek_part_contact()
        except ValueError as err:
          print(err.args)
          raise SystemExit
        else:
          try:
            # args are (distance, effort)
            p.press_incremental_with_force_fb(.003, 0.6)
          except ValueError as err:
            print(err.args)
            raise SystemExit
          else:
            rospy.loginfo("Pressing at position B succeeded")

    # move press back up
    try: 
      p.press_incremental(-.07)
    except ValueError as err:
        print(err.args)
        raise SystemExit

    try:
      forging_client.move_incremental(0,0,0.02)
    except:
      rospy.logerr("Failed to raise part off of lower die")
      raise SystemExit
    
    # press side c
    try:
      forging_client.move_scanning_object_to_pose(seek_down_start_c)
    except:
      rospy.logerr("Failed to reach seek_down_start_c")
      raise SystemExit

    try:
      forging_client.seek_down()
    except ValueError as err:
      print(err.args)
      raise SystemExit
    else:
      try:
        p.seek_part_contact()
      except ValueError as err:
        print(err.args)
        raise SystemExit
      else:
        try:
          # args are (distance, effort)
          p.press_incremental_with_force_fb(.00075, 0.6)
        except ValueError as err:
          print(err.args)
          raise SystemExit
        else:
          rospy.loginfo("Pressing at position C succeeded")

    # move press back up and move up off of bottom die
    try: 
      p.press_incremental(-.07)
    except ValueError as err:
        print(err.args)
        raise SystemExit
      
    try:
      forging_client.move_incremental(0,0,0.02)
    except:
      rospy.logerr("Failed to raise part off of lower die")
      raise SystemExit

    # press side D
    try:
      forging_client.move_scanning_object_to_pose(seek_down_start_d)
    except:
      rospy.logerr("Failed to reach seek_down_start_d")
      raise SystemExit
    try:
      forging_client.seek_down()
    except ValueError as err:
      print(err.args)
      raise SystemExit
    else:
      try:
        p.seek_part_contact()
      except ValueError as err:
        print(err.args)
        raise SystemExit
      else:
        try:
          # args are (distance, effort)
          p.press_incremental_with_force_fb(.00075, 0.6)
        except ValueError as err:
          print(err.args)
          raise SystemExit
        else:
          rospy.loginfo("Pressing at position D succeeded")

    # move press back up
    try: 
      p.press_incremental(-.07)
    except ValueError as err:
        print(err.args)
        raise SystemExit

    try:
      forging_client.move_incremental(0,0,0.02)
    except:
      rospy.logerr("Failed to raise part off of lower die")
      raise SystemExit

  except:
    rospy.logerr("Failed main try block")
    raise SystemExit

if __name__ == "__main__":
  main()
  ForgingClient