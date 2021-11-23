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

/* Author: Phat Do */
/*
  Target:
  1. Simulate moving objects on conveyor
  2. Perform conveyor tracking in ROS
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Bool.h>
#include <cr_msg/Pose.h>
#include <cr_msg/PoseArray.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.

cr_msg::Pose object_pose;
cr_msg::PoseArray object_poses;

// void UpdatePoint(geometry_msgs::Pose &object_pose)
// {

//   object_pose.position.y += conveyor_vel * timer_duration * conveyor_dir;

// }

// //---------------------------------------//
// // Update object position by 100Hz timer
// //---------------------------------------//
// void TimerCallBack(const ros::TimerEvent& e)
// {

//   UpdatePoint(object_poses[0]);
//   UpdatePoint(object_poses[1]);
//   UpdatePoint(object_poses[2]);
//   UpdatePoint(object_poses[3]);
//   UpdatePoint(object_poses[4]);
// }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_publisher");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher object_pose_pub = node_handle.advertise<cr_msg::PoseArray>("/obj_poses_cv", 1000);

  object_pose.position.x = 0.5;
  object_pose.position.y = 0.8;
  object_pose.position.z = 0.05;
  object_pose.type.number = 0;
  object_poses.poses.push_back(object_pose);
  object_pose.position.x = 0.4;
  object_pose.position.y = 1.5;
  object_pose.type.number = 1;
  object_poses.poses.push_back(object_pose);
  object_pose.position.x = 0.6;
  object_pose.position.y = 2.5;
  object_pose.type.number = 2;
  object_poses.poses.push_back(object_pose);
  object_pose.position.x = 0.5;
  object_pose.position.y = 3.3;
  object_pose.type.number = 3;
  object_poses.poses.push_back(object_pose);
  ros::Duration(0.5).sleep();
  object_pose_pub.publish(object_poses);

  while(ros::ok())
   {
      
   }

  return 0;
}

