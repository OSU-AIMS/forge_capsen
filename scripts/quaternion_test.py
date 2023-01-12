import rospy
from tf_conversions import transformations
from tf.transformations import *
from geometry_msgs.msg import Pose, Point, PoseArray

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
    return end_pos

def main():

  pose1 = Pose()
  pose1.position.x = 0.577
  pose1.position.y = -0.205
  pose1.position.z = 0.502
  pose1.orientation.x = 0.91965
  pose1.orientation.y = 0
  pose1.orientation.z = 0
  pose1.orientation.w = 0.3927

  pose2 = pose_from_incremental_move(pose1, 0, 0, 0, -180, 0, 0)

  print("pose2 = {}".format(pose2))

if __name__ == "__main__":
  main()