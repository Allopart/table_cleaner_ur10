/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <stdio.h>
#include <iostream>
#include <string.h>

#define AVOID_OBS true

//using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movable_obstacles");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("my_ur10_limited");
 

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  bool success = false;
  /* This sleep is ONLY to allow Rviz to come up */
  sleep(5.0);

  /*************************************************************/
  // Adding cage to the scenne
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  
  moveit_msgs::CollisionObject collision_object_v1;
  collision_object_v1.header.frame_id = group.getPlanningFrame();
  collision_object_v1.id = "v1";

  moveit_msgs::CollisionObject collision_object_v2;
  collision_object_v2.header.frame_id = group.getPlanningFrame();
  collision_object_v2.id = "v2";
  
  moveit_msgs::CollisionObject collision_object_v3;
  collision_object_v3.header.frame_id = group.getPlanningFrame();
  collision_object_v3.id = "v3";
  
  moveit_msgs::CollisionObject collision_object_v4;
  collision_object_v4.header.frame_id = group.getPlanningFrame();
  collision_object_v4.id = "v4";
  
  /* Define planes to add to the world. */
  shape_msgs::SolidPrimitive box_vertical_1;
  box_vertical_1.type = box_vertical_1.BOX;
  box_vertical_1.dimensions.resize(3);
  box_vertical_1.dimensions[0] = 0.02;
  box_vertical_1.dimensions[1] = 1.0;
  box_vertical_1.dimensions[2] = 1.0;

  shape_msgs::SolidPrimitive box_vertical_2;
  box_vertical_2.type = box_vertical_2.BOX;
  box_vertical_2.dimensions.resize(3);
  box_vertical_2.dimensions[0] = 1.00;
  box_vertical_2.dimensions[1] = 0.02;
  box_vertical_2.dimensions[2] = 1.0;
  
 
  /* Define position of the sticks */
  geometry_msgs::Pose plane_1_pose_1;
  plane_1_pose_1.orientation.w = 1.0;
  plane_1_pose_1.position.x = 0.9;
  plane_1_pose_1.position.y = 0.5;
  plane_1_pose_1.position.z = -0.25;
  
  geometry_msgs::Pose plane_1_pose_2;
  plane_1_pose_2.orientation.w = 1.0;
  plane_1_pose_2.position.x = -0.4;
  plane_1_pose_2.position.y = 0.5;
  plane_1_pose_2.position.z = -0.25;
  
  geometry_msgs::Pose plane_2_pose_1;
  plane_2_pose_1.orientation.w = 1.0;
  plane_2_pose_1.position.x = 0.4;
  plane_2_pose_1.position.y = 1.0;
  plane_2_pose_1.position.z = -0.25;
  
  geometry_msgs::Pose plane_2_pose_2;
  plane_2_pose_2.orientation.w = 1.0;
  plane_2_pose_2.position.x = 0.4;
  plane_2_pose_2.position.y = -0.2;
  plane_2_pose_2.position.z = 0.25;
  
  
  
  /* Create collision object*/
  collision_object_v1.primitives.push_back(box_vertical_1);
  collision_object_v1.primitive_poses.push_back(plane_1_pose_1);
  collision_object_v1.operation = collision_object_v1.ADD;
  
  collision_object_v2.primitives.push_back(box_vertical_1);
  collision_object_v2.primitive_poses.push_back(plane_1_pose_2);
  collision_object_v2.operation = collision_object_v2.ADD;
  
  collision_object_v3.primitives.push_back(box_vertical_2);
  collision_object_v3.primitive_poses.push_back(plane_2_pose_1);
  collision_object_v3.operation = collision_object_v3.ADD;
  
  collision_object_v4.primitives.push_back(box_vertical_2);
  collision_object_v4.primitive_poses.push_back(plane_2_pose_2);
  collision_object_v4.operation = collision_object_v4.ADD;
  
  
  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object_v1);
  collision_objects.push_back(collision_object_v2);
  collision_objects.push_back(collision_object_v3);
  collision_objects.push_back(collision_object_v4);

  
  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  

  ros::shutdown();  
  return 0;
}
