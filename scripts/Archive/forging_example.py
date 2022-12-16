#!/usr/bin/env python

# ROS
import rospy
import numpy as np
import time
import datetime

# ROS Structures
from geometry_msgs.msg import Pose
from capsen_vision.srv import SetProperty, MoveLinkToPose

######################
## Global Variables ##
######################

POSITION_NAMES = [
  'tucked_home',
  'wristup_home',
  'wristup_furnace_front',
  'press_top',
]


#############
## Classes ##
#############

class ForgingClient:

  def __init__(self):
    self._move_to_joint_positions_client = rospy.ServiceProxy(
        '/planner/set_mode_service', SetProperty)

    self._move_link_to_pose_client = rospy.ServiceProxy(
        '/planner/move_link_to_pose_service', MoveLinkToPose)

  def move_home(self):
    response = self._move_to_joint_positions_client(type=0)

    if response.error:
      print("failed to move home: %s"%response.error)

  def move_to_position_names(self):
    # Make a separate call to the service for each position in the list.
    # The service will block this thread until it either fails or finishes
    # moving to the desired location.
    for position_name in POSITION_NAMES:
      response = self._move_to_joint_positions_client(
          type=1, name=position_name)

      if response.error:
        print("failed to move to joint position: %s"%response.error)
        break

  def move_to_scanning_positions(self):
    response = self._move_to_joint_positions_client(type=2)

    if response.error:
      print("failed to move home: %s"%response.error)

  def move_scanning_object_to_pose(self, pose):
    response = self._move_link_to_pose_client(link="object_link", pose=pose)

    if response.error:
      print("failed to move object to pose: %s"%response.error)


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

  forging_client = ForgingClient()

  try:
#    forging_client.move_home()
#    forging_client.move_to_position_names()
#    forging_client.move_to_scanning_positions()
    for i in np.arange(0,5,1):
      pose = Pose()
      pose.position.x = .275 + (.05*i)
      pose.position.y = 0 + (.05*i)
      pose.position.z = 0 + (.05*i)
      pose.orientation.x = 0.028
      pose.orientation.y = .712
      pose.orientation.z = -0.029
      pose.orientation.w = 0.701

      err = forging_client.move_scanning_object_to_pose(pose)
      # time.sleep(2)
      if (err < 0):
        break

  except rospy.ServiceException as e:
    print("service call faild: %s"%e)


if __name__ == "__main__":
  main()
