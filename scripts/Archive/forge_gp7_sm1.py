#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: W.S. Hansen, A.C. Buynak

# Description:
#
#

# Utilities
import os
import sys
import math
import time
import string
from turtle import clear


# ROS
import rospy
import rospkg
from geometry_msgs.msg import Pose, Point, PoseArray
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Temperature
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO
from capsen_vision.srv import SetProperty, MoveLinkToPose
from tf_conversions import transformations
from tf.transformations import *

#####################
# Global Parameters #
#####################

POSITION_NAMES = [
  'tucked_home',
  'wristup_home',
  'wristup_furnace_front',
  'wristup_furnace_pick',
  'wristup_furnace_lift',
  'wristup_furnace_withdrawn',
  'press_top',
]

press_down_target_position = 0.144
press_up_target_position = 0.08
press_down_timeout = 5
press_up_timeout = 5
press_down_tolerance = .01
press_up_tolerance = .01

press_hold_time = 2
forging_temp_low_limit = 350    # deg. C
part_at_forging_temperature = False
forging_complete = False
forging_active = False
state_machine_state = 0
manual_furnace_wait = True
furnace_wait_time_preset_seconds = 3600

class Press():

    def __init__(self):
        self.power_switch_state = None
        self.position = None
        self.raw_temperature = None
        self.adjusted_temperature = None
        self.temperature_alarm_state = None
        self.fork_state = None

        # Enter IR temperature sensor calibration offset
        self.temperature_offset = 100

        # ROS subscribers
        self.position_sub = rospy.Subscriber('/coaliron/joint_states', JointState, self.position_callback)
        self.pss_sub = rospy.Subscriber('/coaliron/power_switch_state', Bool, self.pss_callback)
        self.temp_sub = rospy.Subscriber('/coaliron/temperature1', Temperature, self.temp_callback)
        self.tas_sub = rospy.Subscriber('/coaliron/temperature1_alarm1_state', Bool, self.tas_callback)
        self.fs_sub = rospy.Subscriber('/coaliron/fork_state', Bool, self.fs_callback)

        # ROS publishers
        self.pub = rospy.Publisher('/coaliron/joint_command', Float64,queue_size=10)
        self.rate = rospy.Rate(1)

    def position_callback(self, data):
        self.position = data.position[0]
        # rospy.loginfo(self.position)

    def pss_callback(self, data):
        self.power_switch_state = data
        # rospy.loginfo(self.power_switch_state)

    def temp_callback(self, data):
        self.raw_temperature = data.temperature
        self.adjusted_temperature = float(self.raw_temperature) + self.temperature_offset
        # rospy.loginfo(self.temperature)

    def tas_callback(self, data):
        self.temperature_alarm_state = data
        # rospy.loginfo(self.temperature_alarm_state)

    def fs_callback(self, data):
        self.fork_state = data
        # rospy.loginfo(self.fork_state)

    def publish_once(self, press_target_pos):
        while not rospy.is_shutdown():
            connections = self.pub.get_num_connections()
            rospy.loginfo('Connections: %d', connections)
            if connections > 0:
                self.pub.publish(press_target_pos)
                rospy.loginfo('Published %f', press_target_pos)
                break
            self.rate.sleep()
            
    def press(self, target, tolerance, timeout):
        if self.power_switch_state == False:
            rospy.loginfo('Press failed because power switch is off')
            return False
        elif abs(target - self.position) < tolerance:
            rospy.loginfo('Already at requested location')
            return False
        start_time = time.time()
        self.publish_once(target)
        while (time.time() - start_time) < timeout:
            if abs(target - float(self.position)) < tolerance:
                rospy.loginfo('Successfully pressed to %f', target)
                return True
            else:
                time.sleep(0.1)
        rospy.loginfo('Press timeout before target reached')
        return False

class ForgingClient:

  def __init__(self):
    self._move_to_joint_positions_client = rospy.ServiceProxy(
        '/planner/set_mode_service', SetProperty)

    self._move_link_to_pose_client = rospy.ServiceProxy(
        '/planner/move_link_to_pose_service', MoveLinkToPose)

    self._set_gripper_state_client = rospy.ServiceProxy(
        '/planner/set_gripper_state_service', SetProperty)

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

  def set_gripper_state(self, close):
      response = self._set_gripper_state_client(on_off=close)

      if response.error:
        print("failed to set gripper state: %s"%response.error)

# Initialize
rospy.init_node('forge_gp7_node')
p = Press()
forging_client = ForgingClient()

########
# Main #
########
  
def main():

    # Load Services
    # rospy.loginfo("Waiting for Motoman IO Services...")
    # rospy.wait_for_service('/write_single_io')
    # rospy.wait_for_service('/read_single_io')

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

    rospy.loginfo('Waiting for 1 second')
    time.sleep(1)

    # print("Executing " + __file__)

    # initial startup sequence
    wait_for_part_to_heat()
    retrieve_part_from_furnace()
    move_to_temperature_sensor()
    check_part_temperature()

    # main forging loop
    while (not forging_complete):
        while (part_at_forging_temperature):
            if (not forging_active):
                next_forging_step()

def next_forging_step():
    forging_active = True
    try:
        # forging_client.move_home()
        # forging_client.move_to_position_names()
        # forging_client.move_to_scanning_positions()
        # TODO: write method that accepts pose
        forging_client.move_scanning_object_to_pose()

        return 0

    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
        return 1

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
    q_rot = quaternion_from_euler(drx, dry, drz)
    end_pose.orientation = quaternion_multiply(q_rot, start_pose.orientation)
    return end_pose

def build_pose_array_cog1 (start_pose):
    # expecting start_pose to be type geometry_msgs/PoseStamped with header and point, quaternion representation
    # expecting start_pose to be press location for corner 1
    # expecting move reference frame to be tool frame

    pose_array = PoseArray()
    pose_array.header.frame_id = start_pose.header.frame_id

    dx = 0
    dy = 0
    dz = 0
    drx = 0
    dry = 0
    drz = 90
    corner1 = start_pose.pose
    corner2 = pose_from_incremental_move(corner1,dx,dy,dz,drx,dry,drz)
    corner3 = pose_from_incremental_move(corner2,dx,dy,dz,drx,dry,drz)
    corner4 = pose_from_incremental_move(corner3,dx,dy,dz,drx,dry,drz)

    drz = -22.5
    corner5 = pose_from_incremental_move(corner1,dx,dy,dz,drx,dry,drz)

    drz = 45
    corner6 = pose_from_incremental_move(corner5,dx,dy,dz,drx,dry,drz)
    corner7 = pose_from_incremental_move(corner6,dx,dy,dz,drx,dry,drz)
    corner8 = pose_from_incremental_move(corner7,dx,dy,dz,drx,dry,drz)
    corner9 = pose_from_incremental_move(corner8,dx,dy,dz,drx,dry,drz)
    corner10 = pose_from_incremental_move(corner9,dx,dy,dz,drx,dry,drz)
    corner11 = pose_from_incremental_move(corner10,dx,dy,dz,drx,dry,drz)
    corner12 = pose_from_incremental_move(corner11,dx,dy,dz,drx,dry,drz)
    corner13 = pose_from_incremental_move(corner12,dx,dy,dz,drx,dry,drz)

    pose_array.poses.append(corner1)
    pose_array.poses.append(corner2)
    pose_array.poses.append(corner3)
    pose_array.poses.append(corner4)

    pose_array.poses.append(corner1)
    pose_array.poses.append(corner2)
    pose_array.poses.append(corner3)
    pose_array.poses.append(corner4)

    pose_array.poses.append(corner5)
    pose_array.poses.append(corner6)
    pose_array.poses.append(corner7)
    pose_array.poses.append(corner8)
    pose_array.poses.append(corner9)
    pose_array.poses.append(corner10)
    pose_array.poses.append(corner11)
    pose_array.poses.append(corner12)
    pose_array.poses.append(corner13)

    pose_array.poses.append(corner5)
    pose_array.poses.append(corner6)
    pose_array.poses.append(corner7)
    pose_array.poses.append(corner8)
    pose_array.poses.append(corner9)
    pose_array.poses.append(corner10)
    pose_array.poses.append(corner11)
    pose_array.poses.append(corner12)
    pose_array.poses.append(corner13)

    return pose_array

def move_to_temperature_sensor():
    # make move
    return

def check_part_temperature():
    rospy.loginfo('Part temperature %s degrees C', string(p.adjusted_temperature))
    if(p.adjusted_temperature >= forging_temp_low_limit):
        part_at_forging_temperature = True
    elif(p.adjusted_temperature < forging_temp_low_limit):
        repeat_question = True
        while (repeat_question==True):
            rospy.loginfo('Part below minimum forging temperature.')
            response = str.lower(input('Enter "i" to ignore and continue forging, "s" to scan part, "f" to place part in furnace "or "q" to quit program: '))
            if(response=='i'):
                repeat_question = False
                return
            elif(response=='s'):
                repeat_question = False
                return
            elif(response=='f'):
                repeat_question = False
                return
            elif(response=='q'):
                repeat_question = False
                sys.exit(0)
            else:
                rospy.loginfo('Response not recognized')

        return_part_to_furnace()
        wait_for_part_to_heat()
    return

def retrieve_part_from_furnace():
    open_furnace_door()
    # move to part grasp location
    grip_close()
    time.sleep(1)
    # move to furnace clear location
    close_furnace_door()
    return

def return_part_to_furnace():
    open_furnace_door()
    # move to part grasp location
    grip_open()
    time.sleep(1)
    # move to furnace clear location
    close_furnace_door()
    return

def open_furnace_door():
    input('open furnace door and press Enter')
    return

def close_furnace_door():
    input('close furnace door and press Enter')
    return

def wait_for_part_to_heat():
    furnace_wait_start_time = time.perf_counter()
    if(manual_furnace_wait):
        repeat_question = True
        while (repeat_question == True):
            rospy.loginfo('Waiting for part to reach forging temperature.')
            response = input('Enter "y" to begin forging or "q" to quit program: ')
            if(response=='y'):
                repeat_question = False
            elif(response=='q'):
                repeat_question = False
                sys.exit(0)
                pass
            else:
                rospy.loginfo('Response not recognized')
                pass
    else:
        cancel_furnace_wait = False
        while (not cancel_furnace_wait):
            furnace_wait_time_remaining = furnace_wait_time_preset_seconds - furnace_wait_start_time
            rospy.loginfo('Waiting %s for part to reach forging temperature.', string(furnace_wait_time_remaining), end="\r")
            if (furnace_wait_time_remaining < 0): cancel_furnace_wait=True

def grip_close():
    try:
        forging_client.set_gripper_state(True) # close gripper
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
    time.sleep(1)

def grip_open():
    try:
        forging_client.set_gripper_state(False) # open gripper
    except rospy.ServiceException as e:
        print("service call faild: %s"%e)
    time.sleep(1)

def press_down():
    press_success = p.press(press_down_target_position, press_down_tolerance, press_down_timeout)
    if press_success:
        time.sleep(press_hold_time)

def press_up():
    p.press(press_up_target_position, press_up_tolerance, press_up_timeout)

if __name__ == "__main__":
    # Run Main
    main()
