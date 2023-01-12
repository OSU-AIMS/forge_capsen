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

class ForgingClient:

  def __init__(self):
    self._move_to_joint_positions_client = rospy.ServiceProxy(
        '/planner/set_mode_service', SetProperty)

    self._move_link_to_pose_client = rospy.ServiceProxy(
        '/planner/move_link_to_pose_service', MoveLinkToPose)

    self._set_gripper_state_client = rospy.ServiceProxy(
        '/planner/set_gripper_state_service', SetProperty)

    self._set_io_client = rospy.ServiceProxy(
        '/planner/set_io_service', SetProperty)

    self._set_collision_checking_client = rospy.ServiceProxy(
        '/planner/set_collision_checking_service', SetProperty)

  def set_io(self, name, activate):
    # `name` must match the second column of exactly one of the rows in
    # io_map.csv
    response = self._set_io_client(name=name, on_off=activate)

    if response.error:
      print("failed to set DIO: %s"%response.error)

  def set_collision_checking(self, link_name, allow_collision):
    response = self._set_collision_checking_client(
        name=link_name, on_off=allow_collision)

    if response.error:
      print("failed to set collision checking: %s"%response.error)

  def move_incremental(self, dx, dy, dz):
    try:
      target_pose = self.get_current_pose()
    except ValueError as err:
      print(err.args)
      raise ValueError('move_incremental unable to determine current pose')
    
    target_pose.position.x += dx
    target_pose.position.y += dy
    target_pose.position.z += dz
    try:
      self.move_scanning_object_to_pose(target_pose)
    except ValueError as err:
      print(err.args)
      raise ValueError('move_incremental failed to move robot as expected')

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

# Initialize
rospy.init_node('forge_gp7_node')
forging_client = ForgingClient()

def main():

  rospy.loginfo("Waiting for services")

  # service for moving the scanned object to a pose relative to the robot
  rospy.wait_for_service('/planner/move_link_to_pose_service')

  # activate/deactivate DIO
  rospy.wait_for_service('/planner/set_io_service')

  # enable or disable collision checking for a specified link
  rospy.wait_for_service('/planner/set_collision_checking_service')

  rospy.loginfo("Done waiting for services")

  with h5py.File('IncrementalFormingSimulationTest2.hdf5', 'r') as f:
    dset = f['poses']
    for data in dset:
      print("Pose from hdf5 file: ", data)

      pose1 = Pose()
      pose1.position.x = 0.973 - (data[2] / 1000)
      pose1.position.y = -0.176 - (data[1] / 1000)
      pose1.position.z = 0.200 - (data[0] / 1000)
      pose1.orientation.x = data[3]
      pose1.orientation.y = data[4]
      pose1.orientation.z = data[5]
      pose1.orientation.w = data[6]

      print("Calculated pose: ", pose1)
      time.sleep(5)
      forging_client.move_scanning_object_to_pose(pose1)
      time.sleep(3)

  # pose2 = pose_from_incremental_move(pose1, 0, 0, 0, 90, 0, 0)
  # print("pose2 = {}".format(pose2))

if __name__ == "__main__":
  
  main()