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
from scipy import linalg
from scipy import asarray
from scipy import array_equal
from scipy import negative
from scipy import dot
from scipy import math
from scipy import cross
## END_SUB_TUTORIAL

from visualization_msgs.msg import MarkerArray
from agile_grasp.msg import Grasps
from geometry_msgs.msg import PoseStamped
from time import gmtime, strftime

# Go to home pose
home_pose = geometry_msgs.msg.PoseStamped()
home_pose.header.frame_id = "/world"
home_pose.header.stamp = rospy.Time(0)
home_pose.pose.position.x = 0.0
home_pose.pose.position.y = 0.6
home_pose.pose.position.z = -0.2
home_pose.pose.orientation.x = -0.7
home_pose.pose.orientation.y = 0.07
home_pose.pose.orientation.z = 0.7
home_pose.pose.orientation.w = 0.08

#def go_home():

#  print "============ GO_HOME ============"
#  group.set_pose_target(home_pose.pose, end_effector_link="my_eef")
#  group.go(wait=True)
#  rospy.sleep(5)
#  group.clear_pose_targets()

#def home_pose(my_home_pose):
#  print "============ GO_HOME ============"
#  group.set_pose_target(my_home_pose.pose, end_effector_link="my_eef")
#  group.go(wait=True)
#  rospy.sleep(5)
#  group.clear_pose_targets()

def grasp_callback(my_grasp):
  
  
  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  
  pose_target = geometry_msgs.msg.PoseStamped()
  pose_target.header.stamp = my_grasp.markers[0].header.stamp.secs
  pose_target.header.frame_id = "/camera_rgb_optical_frame"
  pose_target.pose.position.x = my_grasp.markers[0].points[1].x
  pose_target.pose.position.y = my_grasp.markers[0].points[1].y
  pose_target.pose.position.z = my_grasp.markers[0].points[1].z


  ## Convert to quaternion

  u = [1,0,0]
  norm = linalg.norm([my_grasp.markers[0].points[0].x - my_grasp.markers[0].points[1].x, my_grasp.markers[0].points[0].y - my_grasp.markers[0].points[1].y, my_grasp.markers[0].points[0].z - my_grasp.markers[0].points[1].z])
  v = asarray([my_grasp.markers[0].points[0].x - my_grasp.markers[0].points[1].x, my_grasp.markers[0].points[0].y - my_grasp.markers[0].points[1].y, my_grasp.markers[0].points[0].z - my_grasp.markers[0].points[1].z])/norm 

  if (array_equal(u, v)):
    pose_target.pose.orientation.w = 1
    pose_target.pose.orientation.x = 0
    pose_target.pose.orientation.y = 0
    pose_target.pose.orientation.z = 0
  elif (array_equal(u, negative(v))):
    pose_target.pose.orientation.w = 0
    pose_target.pose.orientation.x = 0
    pose_target.pose.orientation.y = 0
    pose_target.pose.orientation.z = 1
  else:
    half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
    pose_target.pose.orientation.w = dot(u, half)
    temp = cross(u, half)
    pose_target.pose.orientation.x = temp[0]
    pose_target.pose.orientation.y = temp[1]
    pose_target.pose.orientation.z = temp[2]
  norm = math.sqrt(pose_target.pose.orientation.x*pose_target.pose.orientation.x + pose_target.pose.orientation.y*pose_target.pose.orientation.y + 
    pose_target.pose.orientation.z*pose_target.pose.orientation.z + pose_target.pose.orientation.w*pose_target.pose.orientation.w)
  if norm == 0:
    norm = 1
  pose_target.pose.orientation.x = pose_target.pose.orientation.x/norm
  pose_target.pose.orientation.y = pose_target.pose.orientation.y/norm
  pose_target.pose.orientation.z = pose_target.pose.orientation.z/norm
  pose_target.pose.orientation.w = pose_target.pose.orientation.w/norm


  print "Timestamp: %d." %pose_target.header.stamp
  print "Position X: %f." % pose_target.pose.position.x
  print "Position Y: %f." % pose_target.pose.position.y
  print "Position Z: %f." % pose_target.pose.position.z
  print "Orientation X: %f." % pose_target.pose.orientation.x
  print "Orientation Y: %f." % pose_target.pose.orientation.y
  print "Orientation Z: %f." % pose_target.pose.orientation.z
  print "Orientation W: %f." % pose_target.pose.orientation.w


  ## Broadcast transform
  br = tf.TransformBroadcaster()
  br.sendTransform((pose_target.pose.position.x, pose_target.pose.position.y, pose_target.pose.position.z), (pose_target.pose.orientation.x, pose_target.pose.orientation.y, pose_target.pose.orientation.z, pose_target.pose.orientation.w), my_grasp.markers[0].header.stamp, "/grasping_target", "/camera_rgb_optical_frame")
  #br.sendTransform((pose_target.pose.position.x, pose_target.pose.position.y, pose_target.pose.position.z), tf.transformations.quaternion_from_euler(my_grasp.grasps[0].axis.x, my_grasp.grasps[0].axis.y, my_grasp.grasps[0].axis.z), my_grasp.header.stamp, "/grasping_target", "/camera_rgb_optical_frame")

  ## Listening to transform
  (trans,rot) = listener.lookupTransform('/world', '/grasping_target', rospy.Time(0))
   

  ## Build new pose
  pose_target_trans = geometry_msgs.msg.PoseStamped()
  pose_target_trans.header.frame_id = "/world"
  pose_target_trans.header.stamp=my_grasp.markers[0].header.stamp
  pose_target_trans.pose.position.x = trans[0]
  pose_target_trans.pose.position.y = trans[1]
  pose_target_trans.pose.position.z = trans[2]
  pose_target_trans.pose.orientation.x = rot[0]
  pose_target_trans.pose.orientation.y = rot[1]
  pose_target_trans.pose.orientation.z = rot[2]
  pose_target_trans.pose.orientation.w = rot[3]
  
  print "NEW POSITION."
  print "Position X: %f." % pose_target_trans.pose.position.x
  print "Position Y: %f." % pose_target_trans.pose.position.y
  print "Position Z: %f." % pose_target_trans.pose.position.z
  print "Orientation X: %f." % pose_target_trans.pose.orientation.x
  print "Orientation Y: %f." % pose_target_trans.pose.orientation.y
  print "Orientation Z: %f." % pose_target_trans.pose.orientation.z
  print "Orientation W: %f." % pose_target_trans.pose.orientation.w
  

  my_grasp_pub.publish(pose_target_trans)

  group.set_pose_target(pose_target_trans.pose, end_effector_link="my_eef")

  

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot


#  group.set_planner_id("RRTstarkConfigDefault")
#  group.allow_replanning(True)


  ## Planning with collision detection can be slow.  Lets set the planning time
  ## to be sure the planner has enough time to plan around the box.  10 seconds
  ## should be plenty.
#  group.set_planning_time(5.0)


  plan1 = group.plan()


  ## Moving to a pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Moving to a pose goal is similar to the step above
  ## except we now use the go() function. Note that
  ## the pose goal we had set earlier is still active 
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is 
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  # Uncomment below line when working with a real robot
#  group.go(wait=True)
  
  print "============ DONE PLANNING ============"
  sys.exit("DONE PLANNING")
  ## Sleep to give Rviz time to visualize the plan. */
  rospy.sleep(5)
  group.clear_pose_targets()

#  go_home()
#  rospy.Subscriber("/home_pose", PoseStamped, home_pose)


if __name__ == '__main__':
  
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('interfacer', anonymous=True)
  listener = tf.TransformListener()
  
  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()
  
  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("my_ur10_limited")
  
  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',     moveit_msgs.msg.DisplayTrajectory)

  # Publisher 
  my_grasp_pub = rospy.Publisher('my_grasps', PoseStamped)
  # Subscriber
  rospy.Subscriber("/dish_grasps/handles_visual", MarkerArray, grasp_callback)
  rospy.Subscriber("/fork_grasps/handles_visual", MarkerArray, grasp_callback)
  rospy.Subscriber("/knife_grasps/handles_visual", MarkerArray, grasp_callback)
  rospy.Subscriber("/glass_grasps/handles_visual", MarkerArray, grasp_callback)
  rospy.Subscriber("/spoon_grasps/handles_visual", MarkerArray, grasp_callback)


  print "============ SPIN ============"
  rospy.spin()
