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
const double tau = 2 * M_PI;

//
namespace rvt = rviz_visual_tools;

// 
const double timer_duration = 0.01; // 100Hz timer
const double conveyor_vel = 0.118; // default = 0.4 m/s
const double conveyor_dir = -1; // 1 means it's the same as direction of y axis, -1 is reversed

// Define planning group, base frame and end effector link
static const std::string PLANNING_GROUP = "manipulator";
static const std::string BASE_FRAME = "base_link";
static const std::string EE_BASE_LINK = "link_6"; 


const double CONVEYOR_WIDTH = 0.55;
const double CONVEYOR_LENGTH = 5.0;
const double CONVEYOR_HEIGHT = 0.815; // default 0.815 m

const double BIN_WIDTH = 0.2;
const double BIN_LENGTH = 0.2;
const double BIN_HEIGHT = 0.3;

bool received_obj = 0;
int counter_store = 0;
int counter_track = 0;

//
geometry_msgs::Pose object_pose;
geometry_msgs::Pose object_poses[10];

//geometry_msgs::PoseArray object_poses[10];

void UpdatePoint(geometry_msgs::Pose &object_pose)
{
  // Update the object position
  object_pose.position.y += conveyor_vel * timer_duration * conveyor_dir;
  //ROS_INFO("Point is updated %f", object_pose.position.y);
}

//---------------------------------------//
// Update object position by 100Hz timer
//---------------------------------------//
void TimerCallBack(const ros::TimerEvent& e)
{
  //ROS_INFO("Point is updated");
  //UpdatePoint(object_pose);
  UpdatePoint(object_poses[0]);
  UpdatePoint(object_poses[1]);
  UpdatePoint(object_poses[2]);
  UpdatePoint(object_poses[3]);
  UpdatePoint(object_poses[4]);
}


void ObjectCallBack(const geometry_msgs::Pose::ConstPtr &msg)
{
  object_pose.position = msg->position;
  received_obj = 1;
  ROS_INFO("Object received.");
  //ROS_INFO("Point is updated %f", msg->position.y);
}

void ObjectPosesCallBack(const geometry_msgs::Pose::ConstPtr &msg)
{
  object_poses[counter_store].position = msg->position;
  counter_store++;
  if (counter_store > 4) counter_store = 0;
  received_obj = 1;
  ROS_INFO("Object received.");
  //ROS_INFO("Point is updated %f", msg->position.y);
}

// void ObjectPosesCallBack(const geometry_msgs::PoseArray::ConstPtr &msg)
// {
//   object_poses->poses[0].position = msg->poses[0].position;
//   object_poses->poses[0].orientation = msg->poses[0].orientation;
//   object_poses->poses[1].position = msg->poses[1].position;
//   object_poses->poses[1].orientation = msg->poses[1].orientation;
//   object_poses->poses[2].position = msg->poses[2].position;
//   object_poses->poses[2].orientation = msg->poses[2].orientation;
//   received_obj = 1;
//   // //ROS_INFO("Point is updated %f", msg->position.y);
// }

void DoneCallback(const std_msgs::Bool::ConstPtr& msg)
{
  counter_track++;
  if (counter_track > 4) counter_track = 0;
}


//------------------------------//
// Add some collision objects
//------------------------------//
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  int object_number = 4;
  collision_objects.resize(object_number);

  // Define conveyor
  collision_objects[0].id = "conveyor";
  collision_objects[0].header.frame_id = BASE_FRAME;
  // Define conveyor dimensions
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);

  collision_objects[0].primitives[0].dimensions[0] = CONVEYOR_WIDTH; // conveyor width
  collision_objects[0].primitives[0].dimensions[1] = CONVEYOR_LENGTH; // conveyor length
  collision_objects[0].primitives[0].dimensions[2] = CONVEYOR_HEIGHT; // conveyor height
  // Define conveyor pose in the robot frame
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = CONVEYOR_LENGTH/2 - CONVEYOR_LENGTH/2;
  collision_objects[0].primitive_poses[0].position.z = (-CONVEYOR_HEIGHT/2) - (0.025) - 0.3; // 0.025 = robot base  - conveyor height = 0.84 - 0.815
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[0].operation = collision_objects[0].ADD;

  // Define scrap bin 1
  collision_objects[1].id = "scrap_bin1";
  collision_objects[1].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = BIN_WIDTH; // bin width
  collision_objects[1].primitives[0].dimensions[1] = BIN_LENGTH; // bin length
  collision_objects[1].primitives[0].dimensions[2] = BIN_HEIGHT; // bin height
  // Define scrap bin pose in the robot frame
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.94;
  collision_objects[1].primitive_poses[0].position.y = -0.22;
  collision_objects[1].primitive_poses[0].position.z = BIN_HEIGHT/2 - (BIN_HEIGHT);
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[1].operation = collision_objects[1].ADD;

  // Define scrap bin 2
  collision_objects[2].id = "scrap_bin2";
  collision_objects[2].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = BIN_WIDTH; // bin width
  collision_objects[2].primitives[0].dimensions[1] = BIN_LENGTH; // bin length
  collision_objects[2].primitives[0].dimensions[2] = BIN_HEIGHT; // bin height
  // Define scrap bin pose in the robot frame
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.94;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = BIN_HEIGHT/2 - (BIN_HEIGHT);
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[2].operation = collision_objects[2].ADD;

  // Define scrap bin 3
  collision_objects[3].id = "scrap_bin3";
  collision_objects[3].header.frame_id = BASE_FRAME;
  // Define scrap bin dimensions
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = BIN_WIDTH; // bin width
  collision_objects[3].primitives[0].dimensions[1] = BIN_LENGTH; // bin length
  collision_objects[3].primitives[0].dimensions[2] = BIN_HEIGHT; // bin height
  // Define scrap bin pose in the robot frame
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.94;
  collision_objects[3].primitive_poses[0].position.y = 0.22;
  collision_objects[3].primitive_poses[0].position.z = BIN_HEIGHT/2 - (BIN_HEIGHT);
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  //
  collision_objects[3].operation = collision_objects[3].ADD;

  // // Define object
  // collision_objects[2].id = "object";
  // collision_objects[2].header.frame_id = BASE_FRAME;
  // // Define scrap bin dimensions
  // collision_objects[2].primitives.resize(1);
  // collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  // collision_objects[2].primitives[0].dimensions.resize(3);
  // collision_objects[2].primitives[0].dimensions[0] = 0.05;
  // collision_objects[2].primitives[0].dimensions[1] = 0.1;
  // collision_objects[2].primitives[0].dimensions[2] = 0.05;
  // // Define scrap bin pose in the robot frame
  // collision_objects[2].primitive_poses.resize(1);
  // collision_objects[2].primitive_poses[0].position.x = 0.45;
  // collision_objects[2].primitive_poses[0].position.y = -2.5;
  // collision_objects[2].primitive_poses[0].position.z = 0.15;
  // tf2::Quaternion orientation;
  // orientation.setRPY(0, 0, 0); // tau = 2*pi;
  // collision_objects[2].primitive_poses[0].orientation = tf2::toMsg(orientation);
  // //
  // ROS_INFO("Attach gripper on the eef");
  // collision_objects[2].operation = collision_objects[2].ADD;

  // Add the collision object into the world
  planning_scene_interface.addCollisionObjects(collision_objects);
  ROS_INFO("Added collision objects into the world");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irb1200_object_publisher");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Construct planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Wait for starting up the planning scene
  ros::WallDuration(3.0).sleep();

  // Add collision objects
  addCollisionObjects(planning_scene_interface);

  // Planning scene publisher
  // ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  // while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  // {
  //   ros::WallDuration(0.1).sleep();
  // }
  // moveit_msgs::PlanningScene planning_scene;
  // moveit_msgs::CollisionObject moving_object;
  // moving_object.id = "object";
  // moving_object.header.frame_id = BASE_FRAME;
  
  //
  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
  visual_tools.loadRemoteControl();
  // visual_tools.prompt("Press 'next' add an object");

  // Start a 100Hz timer to update object position
  ros::Timer timer = node_handle.createTimer(ros::Duration(timer_duration), TimerCallBack);
  ROS_INFO("100Hz Timer has started.");

  // Create service client to request picking a coming object
  // ros::ServiceClient object_pose_client = node_handle.serviceClient<cr_waste_sorting_robot::PickMovingObject>("picked_object");
  // cr_waste_sorting_robot::PickMovingObject initial_pose; // will be the position from vision system after filtering
  // initial_pose.request.object_pose = object_pose;
  // if (object_pose_client.call(initial_pose))
  // {
  //   if (initial_pose.response.result)
  //   {
  //     // do sth if picking successfully
  //   }
  //   else
  //   {
  //     // do sth if picking unsuccessfully
  //   }
  // }

  // Object pose publisher
  ros::Publisher object_pose_publisher = node_handle.advertise<geometry_msgs::Pose>("moving_object", 1);

  //Initial object pose
  // double object_height = 0.05;
  // object_pose.position.x = 0.45; // 0.45 =  center of conveyor
  // object_pose.position.y = -1.2 * conveyor_dir; // starting point of conveyor
  // object_pose.position.z = object_height - (0.025);
  // object_pose.orientation.w = 1;

  // Visualization
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.9;
  std::stringstream str_conveyor_vel;
  str_conveyor_vel << "Conveyor speed: " << conveyor_vel << " m/s";
  
  // bool object_coming;

  ros::Subscriber object_pose_subscriber = node_handle.subscribe("object_detection_cv", 1, ObjectCallBack);
  ros::Subscriber object_poses_subscriber = node_handle.subscribe("object_poses_cv", 1, ObjectPosesCallBack);

  ros::Subscriber robot_done = node_handle.subscribe("robot_done", 1, DoneCallback);

  // while(ros::ok())
  // {
  //   while(object_pose.position.y > 0.1 * conveyor_dir)
  //   {
  //     // Update object pose
  //     object_pose_publisher.publish(object_pose);
  //     visual_tools.deleteAllMarkers();
  //     visual_tools.publishAxis(object_pose, 0.1);
  //     visual_tools.publishText(text_pose, str_conveyor_vel.str(), rvt::WHITE, rvt::XLARGE);
  //     visual_tools.trigger();

  //     // //
  //     // moving_object.primitive_poses.clear();
  //     // moving_object.primitive_poses.push_back(object_pose);
  //     // moving_object.operation = moving_object.MOVE;

  //     // // 
  //     // planning_scene.world.collision_objects.clear();
  //     // planning_scene.world.collision_objects.push_back(moving_object);
  //     // planning_scene.is_diff = true;
  //     // planning_scene_diff_publisher.publish(planning_scene);
      
  //     // 100Hz publisher
  //     ros::Rate(100).sleep();
  //   }

  //   // Update object pose, random from 0.3 to 0.6 m
  //   object_pose.position.x = 0.3 + (rand() % 100 + 1)*0.3/100;
  //   object_pose.position.y = -1.2 * conveyor_dir; // starting point of conveyor

  //   // if (object_pose.position.x < 0.3)
  //   // {
  //   //   object_pose.position.x = 0.6;
  //   // }
  // }

  while(ros::ok())
  {
    switch (received_obj){
      case 0:
        break;
      case 1:
        while(object_poses[counter_track].position.y > -0.1)
        {
          // Update object pose
          object_pose_publisher.publish(object_poses[counter_track]);
          visual_tools.deleteAllMarkers();
          visual_tools.publishAxis(object_poses[0], 0.1);
          visual_tools.publishAxis(object_poses[1], 0.1);
          visual_tools.publishAxis(object_poses[2], 0.1);
          visual_tools.publishAxis(object_poses[3], 0.1);
          visual_tools.publishAxis(object_poses[4], 0.1);

          visual_tools.publishText(text_pose, str_conveyor_vel.str(), rvt::WHITE, rvt::XLARGE);
          visual_tools.trigger();

          ros::Rate(100).sleep();
        }
        // Update object pose
        //object_pose.position.x -= 0.02; // 0.45 =  center of conveyor
        //object_pose.position.y = -1.2; // starting point of conveyor

        // if (object_pose.position.x < 0.3)
        // {
        //   object_pose.position.x = 0.6;
        // }
        break;
    }
  }

  return 0;
}

