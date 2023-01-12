#!/usr/bin/env python

import time

from geometry_msgs.msg import Pose
import rospy

from capsen_vision.srv import SetProperty, MoveLinkToPose


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

  def move_home(self):
    response = self._move_to_joint_positions_client(type=0)

    if response.error:
      print("failed to move home: %s"%response.error)

  def move_to_position_names(self):
    # Make a separate call to the service for each position in the list.
    for position_name in POSITION_NAMES:
      response = self._move_to_joint_positions_client(
          type=1, name=position_name)

      if response.error:
        print("failed to move to joint position: %s"%response.error)
        break

  def move_to_scanning_positions(self):
    response = self._move_to_joint_positions_client(type=2, val_a=50)

    if response.error:
      print("failed to move to scanning positions: %s"%response.error)

  def move_scanning_object_to_pose(self):
    pose = Pose()
    pose.position.x = .6;
    pose.position.y = -.2;
    pose.position.z = .2;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1.;

    response = self._move_link_to_pose_client(link="object_link", pose=pose)

    if response.error:
      print("failed to move object to pose: %s"%response.error)

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


def main():
  # The param `blocking_service` in planner.yaml decides whether or not the
  # service calls to move the robot will block the client thread until it
  # either fails or finishes moving.

  # service for moving to pre-recorded joint positions
  # type = 0: move to home position recorded in planner.yaml
  # type = 1: move to a position name which exists in action_locations.csv
  # type = 2: move a series of scanning positions and generate the mesh.
  rospy.wait_for_service('/planner/set_mode_service')

  # service for moving the scanned object to a pose relative to the robot
  rospy.wait_for_service('/planner/move_link_to_pose_service')

  # activate/deactivate DIO
  rospy.wait_for_service('/planner/set_io_service')

  # enable or disable collision checking for a specified link
  rospy.wait_for_service('/planner/set_collision_checking_service')

  forging_client = ForgingClient()

  try:
    forging_client.set_io("AIR", True) # close gripper
    time.sleep(2)
    forging_client.set_io("AIR", False) # open gripper

    time.sleep(2)

    forging_client.set_io("DOOR", True) # open door
    time.sleep(2)
    forging_client.set_io("DOOR", False) # close door

  except rospy.ServiceException as e:
    print("service call failed: %s"%e)


if __name__ == "__main__":
  main()
