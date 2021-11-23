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

//#include <tf2_cr_msg/tf2_cr_msg.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <cr_msg/Pose.h>
#include <cr_msg/PoseArray.h>
#include <math.h> 
#include <stdio.h>

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

int counter_track = 0;

//
geometry_msgs::Pose object_pose;

cr_msg::Pose unsorted_poses[100];
cr_msg::Pose sorted_poses[100];
//cr_msg::Pose object_poses[10];

cr_msg::PoseArray obj_poses;

void UpdatePoint(cr_msg::Pose &object_pose)
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
  //UpdatePoint(object_poses[0]);
  //UpdatePoint(object_poses[1]);
  //UpdatePoint(object_poses[2]);
  //UpdatePoint(object_poses[3]);
 
  //UpdatePoint(object_poses[4]);
}


// void ObjectCallBack(const cr_msg::Pose::ConstPtr &msg)
// {
//   object_pose.position = msg->position;
//   received_obj = 1;
//   ROS_INFO("Object received.");
//   //ROS_INFO("Point is updated %f", msg->position.y);
// }

// void ObjectPosesCallBack(const cr_msg::Pose::ConstPtr &msg)
// {
//   object_poses[counter_store].position = msg->position;
//   counter_store++;
//   if (counter_store > 4) counter_store = 0;
//   received_obj = 1;
//   ROS_INFO("Object received.");
//   //ROS_INFO("Point is updated %f", msg->position.y);
// }

int c_unsorted = 0; //to transverse in unsorted array
int act_index_unsorted = 0; //to check HOW MANY are active in unsorted array

void PosesCallBack(const cr_msg::Pose::ConstPtr &msg)
{
  unsorted_poses[c_unsorted].position = msg->position;
  unsorted_poses[c_unsorted].orientation = msg->orientation;
  unsorted_poses[c_unsorted].type = msg->type;
  //ROS_INFO("KKKKKK %f", unsorted_poses[0].position.y);
  c_unsorted++;
  act_index_unsorted += 1;
  received_obj = 1;

  
  //ROS_INFO("Point is updated %f",  object_poses[counter_store].position.y);

}


int c_sort = 0; //to transverse in sorted array
int act_index_sorted = 0; //to check to which index is active in sorted array
int current_bin = 1; //fake initial position
cr_msg::Pose current_object; //fake initial position
float min_distance = 10000.0;
int min_index = 0; //index of current minimum distance
float current_distance = 0;
float min_exec_time = 0;
float avg_end_spd = 0.4; 

void TimerSorting(const ros::TimerEvent& e)
{
  if (act_index_unsorted > 0)
  {
    //printf ("act_index_unsorted: %d \n", act_index_unsorted);
    switch (current_bin){
      case 0:
        current_object.position.x = 0.1;
        current_object.position.y = 0.2;
      break;
      case 1:
        current_object.position.x = 0.1;
        current_object.position.y = -0.2;
      break;
      case 2:
        current_object.position.x = 0.05;
        current_object.position.y = 0.4;
      break;
      case 3:
        current_object.position.x = 0.05;
        current_object.position.y = -0.4;
      break;
    }

    for (int i = 0; i < act_index_unsorted; i++)
    {
      min_distance = 10000.0; //Reset min
      printf ("act_index_unsorted: %d \n", act_index_unsorted);
      for (int j = 0; j < act_index_unsorted; j++)
      {
        printf("%f\n",unsorted_poses[j].position.y);
      }
      printf("BREAK\n");


      for (int j = 0; j < act_index_unsorted; j++) //Compare all element in unsorted array up to current act_index_unsorted
      {
        current_distance = sqrt(pow(unsorted_poses[j].position.x - current_object.position.x,2.0)+pow(unsorted_poses[j].position.y - current_object.position.y,2.0));
        if (current_distance < min_distance) 
        {
          min_distance = current_distance;
          min_index = j;
        }
      }
      //printf("%f\n",unsorted_poses[min_index].position.y);
      //Assign min element to sorted array
      sorted_poses[c_sort] = unsorted_poses[min_index]; 
      c_sort++;

      //Delete the min element in unsorted array;
      for (int j = min_index; j < act_index_unsorted; j++) 
      {
          unsorted_poses[j] = unsorted_poses[j+1];
      }

      min_exec_time = (min_distance*2)/avg_end_spd; //This is WRONG, needs to adjust later
      act_index_unsorted--; //Decrease by one position

      //Update (foresighted) position of remaining elements
      for (int j = 0; j < act_index_unsorted; j++) 
      {
          unsorted_poses[j].position.y += conveyor_vel * min_exec_time * conveyor_dir;
      }

    }

    // for (int i = 0; i < c_sort; i++)
    // {
    //   printf("%f\n",sorted_poses[i].position.y);
    // }

  }
}


void DoneCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // counter_track++;
  // if (counter_track > 4) counter_track = 0;
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
  ros::Timer timer_position = node_handle.createTimer(ros::Duration(timer_duration), TimerCallBack);
  ros::Timer timer_sorting = node_handle.createTimer(ros::Duration(7), TimerSorting);

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
  ros::Publisher object_pose_publisher = node_handle.advertise<cr_msg::Pose>("moving_obj", 10);

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

  //ros::Subscriber object_pose_subscriber = node_handle.subscribe("object_detection_cv", 1, ObjectCallBack);
  //ros::Subscriber object_poses_subscriber = node_handle.subscribe("object_poses_cv", 1, ObjectPosesCallBack);
  ros::Subscriber poses_subscriber = node_handle.subscribe("/obj_poses_cv", 1, PosesCallBack);
  ros::Subscriber robot_done = node_handle.subscribe("robot_done", 1, DoneCallback);


  while(ros::ok())
  {
    

    switch (received_obj){
      case 0:
        break;
      case 1:
        while(unsorted_poses[counter_track].position.y > -5)
        {
          // Update object pose
          //object_pose_publisher.publish(unsorted_poses[counter_track]);
          //ROS_INFO("%d", c_unsorted);
          visual_tools.deleteAllMarkers();

          // object_pose.position.x = object_poses[0].position.x;
          // object_pose.position.y = object_poses[0].position.y;
          // object_pose.position.z = object_poses[0].position.z;
          // visual_tools.publishAxis(object_pose, 0.1);

          // object_pose.position.x = object_poses[1].position.x;
          // object_pose.position.y = object_poses[1].position.y;
          // object_pose.position.z = object_poses[1].position.z;
          // visual_tools.publishAxis(object_pose, 0.1);
          
          // object_pose.position.x = object_poses[2].position.x;
          // object_pose.position.y = object_poses[2].position.y;
          // object_pose.position.z = object_poses[2].position.z;
          // visual_tools.publishAxis(object_pose, 0.1);

          // object_pose.position.x = object_poses[3].position.x;
          // object_pose.position.y = object_poses[3].position.y;
          // object_pose.position.z = object_poses[3].position.z;
          // visual_tools.publishAxis(object_pose, 0.1);

          //visual_tools.publishAxis(object_poses[1], 0.1);
          //visual_tools.publishAxis(object_poses[2], 0.1);
          //visual_tools.publishAxis(object_poses[3], 0.1);
          //visual_tools.publishAxis(object_poses[4], 0.1);

          visual_tools.publishText(text_pose, str_conveyor_vel.str(), rvt::WHITE, rvt::XLARGE);
          visual_tools.trigger();

          ros::Rate(100).sleep();
        }

        break;
    }
  }

  return 0;
}

