
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


int main(int argc, char **argv)
{
  ros::init (argc, argv, "environment_setup");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_time(3.0);
  sleep_time.sleep();
  sleep_time.sleep();
  

// BEGIN_TUTORIAL
// 
// ROS API
// ^^^^^^^
// The ROS API to the planning scene publisher is through a topic interface
// using "diffs". A planning scene diff is the difference between the current 
// planning scene (maintained by the move_group node) and the new planning 
// scene desired by the user. 
//
// Advertise the required topic
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Note that this topic may need to be remapped in the launch file 
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

// Define the attached object message
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// We will use this message to add or 
// subtract the object from the world 
// and to attach the object to the robot
  moveit_msgs::AttachedCollisionObject scaffold_object;
  moveit_msgs::AttachedCollisionObject table_object;
  moveit_msgs::AttachedCollisionObject kinect_object;
  scaffold_object.link_name = "scaffold_link";
  table_object.link_name = "table_link";
  kinect_object.link_name = "kinect_link";
  /* The header must contain a valid TF frame*/
  scaffold_object.object.header.frame_id = "scaffold_link";
  table_object.object.header.frame_id = "table_link";
  kinect_object.object.header.frame_id = "kinect_link";
  /* The id of the object */
  scaffold_object.object.id = "scaffold";
  table_object.object.id = "table";
  kinect_object.object.id = "kinect";
  /* A default pose */
  geometry_msgs::Pose pose_scaffold;
  pose_scaffold.orientation.w = 1.0;
  geometry_msgs::Pose pose_table;
  pose_table.orientation.w = 1.0;
  geometry_msgs::Pose pose_kinect;
  pose_kinect.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive_scaffold;
  primitive_scaffold.type = primitive_scaffold.BOX;
  primitive_scaffold.dimensions.resize(3);
  primitive_scaffold.dimensions[0] = 0.9;
  primitive_scaffold.dimensions[1] = 0.1;
  primitive_scaffold.dimensions[2] = 0.75;
  shape_msgs::SolidPrimitive primitive_table;
  primitive_table.type = primitive_table.BOX;
  primitive_table.dimensions.resize(3);
  primitive_table.dimensions[0] = 0.02;
  primitive_table.dimensions[1] = 0.7;
  primitive_table.dimensions[2] = 0.9;
  shape_msgs::SolidPrimitive primitive_kinect;
  primitive_kinect.type = primitive_kinect.BOX;
  primitive_kinect.dimensions.resize(3);
  primitive_kinect.dimensions[0] = 0.06;
  primitive_kinect.dimensions[1] = 0.12;
  primitive_kinect.dimensions[2] = 0.06;

  scaffold_object.object.primitives.push_back(primitive_scaffold);
  scaffold_object.object.primitive_poses.push_back(pose_scaffold);
  table_object.object.primitives.push_back(primitive_table);
  table_object.object.primitive_poses.push_back(pose_table);
  kinect_object.object.primitives.push_back(primitive_kinect);
  kinect_object.object.primitive_poses.push_back(pose_kinect);

// Note that attaching an object to the robot requires 
// the corresponding operation to be specified as an ADD operation
  scaffold_object.object.operation = scaffold_object.object.ADD;
  table_object.object.operation = table_object.object.ADD;
  kinect_object.object.operation = kinect_object.object.ADD;

// Add an object into the environment
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Add the object into the environment by adding it to 
// the set of collision objects in the "world" part of the 
// planning scene. Note that we are using only the "object" 
// field of the attached_object message here.
  ROS_INFO("Adding the object into the world at the location of the scaffold_link.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(scaffold_object.object);
  planning_scene.world.collision_objects.push_back(table_object.object);
  planning_scene.world.collision_objects.push_back(kinect_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_time.sleep();

  ros::shutdown();
  return 0;
}

