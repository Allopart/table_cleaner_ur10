#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tf
from geometry_msgs.msg import PoseStamped

def  talker():
  # Publisher 
  home_pub = rospy.Publisher('home_pose', PoseStamped, queue_size=10)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    ## Go to home pose
    home_pose = geometry_msgs.msg.PoseStamped()
    home_pose.header.frame_id = "/world"
    home_pose.header.stamp = rospy.Time.now()
    home_pose.pose.position.x = 0.0
    home_pose.pose.position.y = 0.6
    home_pose.pose.position.z = -0.2
    home_pose.pose.orientation.x = 0.74
    home_pose.pose.orientation.y = -0.09
    home_pose.pose.orientation.z = -0.66
    home_pose.pose.orientation.w = 0.0
  
    home_pub.publish(home_pose)
    rate.sleep()

def move_home():
  
  print "----------- HERE -----------"
  home_pub = rospy.Publisher('home_pose', PoseStamped, queue_size=10)

  ## Go to home pose
  home_pose = geometry_msgs.msg.PoseStamped()
  home_pose.header.frame_id = "/world"
  home_pose.header.stamp = rospy.Time.now()
  home_pose.pose.position.x = 0.0
  home_pose.pose.position.y = 0.6
  home_pose.pose.position.z = -0.2
  home_pose.pose.orientation.x = 0.74
  home_pose.pose.orientation.y = -0.09
  home_pose.pose.orientation.z = -0.66
  home_pose.pose.orientation.w = 0.0
  
  home_pub.publish(home_pose)

  group.allow_replanning(True)
  group.set_pose_target(home_pose.pose, end_effector_link="my_eef")
  plan1 = group.plan()
#  group.go(wait=True)
  rospy.sleep(10)



if __name__ == '__main__':
 
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('interfacer', anonymous=True)
  listener = tf.TransformListener()

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("my_ur10_limited")
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',     moveit_msgs.msg.DisplayTrajectory,queue_size=10)


  talker()
#  move_home()
 
  rospy.spin()

  
  







