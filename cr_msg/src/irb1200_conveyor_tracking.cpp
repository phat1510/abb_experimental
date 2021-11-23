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
/* Description:  */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/MoveGroupSequenceAction.h>
#include <moveit_msgs/GetMotionSequence.h>

#include <std_msgs/Bool.h>

#include <cr_msg/Pose.h>
#include <cr_msg/PoseArray.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// Utility functions for displaying and debugging data in Rviz via published markers
namespace rvt = rviz_visual_tools;

// // Object postion from request
// geometry_msgs::Pose object_pose;

//
const double working_conveyor_vel = 0.28; // first test: 0.112m/s

// void ObjectCallBack(const geometry_msgs::Pose::ConstPtr &msg)
// {
//   object_pose.position = msg->position;
//   object_pose.orientation = msg->orientation;
//   //ROS_INFO("Point is updated %f", msg->position.y);
// }

//----------- Chuong-new-OPEN -----------//
// cr_msg::Pose obj_pose_type;
// void PoseCallBack(const cr_msg::Pose::ConstPtr &msg)
// {
//   obj_pose_type.position = msg->position;
//   obj_pose_type.orientation = msg->orientation;
//   obj_pose_type.type = msg->type;
//   ROS_INFO("Point is updated %f", msg->position.y);
// }

cr_msg::Pose object_pose;
void ObjectCallBack(const cr_msg::Pose::ConstPtr &msg)
{
  object_pose.position = msg->position;
  object_pose.orientation = msg->orientation;
  object_pose.type = msg->type;
  //ROS_INFO("Point is updated %f", msg->position.y);
  
}

//----------- Chuong-new-CLOSE -----------//



// bool CheckPose(geometry_msgs::Pose &pose)
// {
//   double x = pose.position.x;
//   double y = pose.position.y;
//   double z = pose.position.z;

//   if ( x < 0.67 && x > 0.33 && y > 0.6 && z < 0.1)
//     return true;
//   else 
//     return false;
// }
//----------- Chuong-new-OPEN -----------//


bool CheckPose(cr_msg::Pose &pose)
{
  double x = pose.position.x;
  double y = pose.position.y;
  double z = pose.position.z;

  if ( x < 0.67 && x > 0.33 && y > 0.6 && z < 0.1)
    return true;
  else 
    return false;
}

//----------- Chuong-new-CLOSE -----------//

int main(int argc, char** argv)
{
  // ROS setup
  // ---------
  // Create the node
  ros::init(argc, argv, "irb1200_conveyor_tracking");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ------------------------------------------------------------------//
  //-----------Setup robot model and planning pipeline
  // ------------------------------------------------------------------//

  // Define planning group, base frame and end effector link
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string BASE_FRAME = "base_link";
  static const std::string EE_BASE_LINK = "link_6"; // The link end-effector will be attached on
  static const std::string TOOL_LINK = "tool0"; // Working point

  // Load the robot description on ROS parameter server
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
  
  // Construct a robot model, which contains the robot's kinematic info
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Construct a planning scene from the robot model loader
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  // 
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Get robot state
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  
  // Create pointer to the planning group
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("manipulator");

  // Setup planning pipeline using planning plugin and request adapter on ROS parameter server
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, 
      "/move_group/planning_pipelines/pilz_industrial_motion_planner/planning_plugin", 
      "/move_group/planning_pipelines/pilz_industrial_motion_planner/request_adapters"));

  moveit_visual_tools::MoveItVisualTools visual_tools(BASE_FRAME);
  // Visualization
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.9;
  visual_tools.publishText(text_pose, "IRB1200 Conveyor Tracking", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Display base link coordinate
  geometry_msgs::Pose base_link;
  base_link.orientation.w = 1;
  base_link.position.x = 0;
  base_link.position.y = 0;
  base_link.position.z = 0;
  visual_tools.publishAxis(base_link, 0.25);
  visual_tools.trigger();

  // Start the demo
  ROS_INFO("Setup finished. Ready to start...");
  //visual_tools.prompt("Press 'next' to go to home pose");

  // ------------------------------------------------------------------//
  //-----------Setup robot's initial position
  // ------------------------------------------------------------------//
  
  // Get current joint position directly from  /joint_states topic and set for robot start state
  sensor_msgs::JointStateConstPtr joint_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
  robot_state->setJointGroupPositions(joint_model_group,joint_values->position);

  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // A tolerance of 0.01 m is specified in position and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // Home position (bin's center position)
  geometry_msgs::PoseStamped home_pose;
  home_pose.header.frame_id = "base_link";
  home_pose.pose.position.x = 0.55; // tested max value 0.783
  home_pose.pose.position.y = 0;
  home_pose.pose.position.z = 0.3; // tested max value 0.399
  tf2::Quaternion orientation;
  orientation.setRPY(0, tau/4, 0); // tau = 2*pi;
  home_pose.pose.orientation = tf2::toMsg(orientation);
  //home_pose.pose.orientation.w = 1.0;

  // Define name of planning group
  req.group_name = "manipulator";
  req.planner_id = "PTP";
  req.allowed_planning_time = 5;
  req.max_velocity_scaling_factor = 0.2;
  req.max_acceleration_scaling_factor = 1;
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Set constraints for pose goal
  ROS_INFO("Construct goal constraints.");
  moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints(TOOL_LINK, home_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);
  
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(lscene, req, res);
    ROS_INFO("Generated path");
  }

  // Now, call the pipeline and check whether planning was successful.
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  //
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Visualize the trajectory
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  //visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
  joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' to execute");

  // Execute the plan and update robot joint states
  move_group_interface.execute(response.trajectory);
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  
  // Create service server to pick the requested object
  // ros::ServiceServer object_pose_server = 
  //         node_handle.advertiseService("picked_object", PickMovingObject);

  // Subscribe object position
  // ros::Subscriber object_pose_subscriber = node_handle.subscribe("moving_object", 1, PoseCallBack);

  // Define velocity values (m/s)
  const double max_lin_vel = 1.5;
  const double max_conv_vel = 1.2;
  const double down_speed = 0.6;
  const double pick_up_speed = 0.8;
  const double gripping_time = 0.8; // per VGG10 datasheet

  // Create a service client
  ros::ServiceClient _planner_client = node_handle.serviceClient<moveit_msgs::GetMotionSequence>("plan_sequence_path");
  moveit_msgs::GetMotionSequence planner_srv;

  //----------- Chuong-new-OPEN -----------//
  ros::Publisher robot_done = node_handle.advertise<std_msgs::Bool>("robot_done", 10);
  std_msgs::Bool done;
  done.data = true;
  ros::Subscriber object_pose_subscriber = node_handle.subscribe("moving_obj", 10, ObjectCallBack);

  //ros::Subscriber obj_type_pose_sub = node_handle.subscribe("moving_object", 1, ObjectCallBack);

  //----------- Chuong-new-CLOSE -----------//


  // ------------------------------------------------------------------//
  //-----------Object tracking loop
  // ------------------------------------------------------------------//
  while(ros::ok())
  {
    // Wait for an valid object
    while(!CheckPose(object_pose))
    {
      ros::Duration(0.1).sleep();
    }

    // ------------------------------------------------------------------//
    //-----------Go to start point of tracking line
    // ------------------------------------------------------------------//

    ROS_INFO("Construct tracking point goal");
    geometry_msgs::PoseStamped tracking_point;
    double bound_radius = 0.68; // default = 0.66
    tracking_point.header.frame_id = "base_link";
    tracking_point.pose.position.x = object_pose.position.x; // get the x coordinate of object
    tracking_point.pose.position.y = 
      sqrt(bound_radius * bound_radius - tracking_point.pose.position.x * tracking_point.pose.position.x); 
    if (tracking_point.pose.position.x < 0.6)
    {
      tracking_point.pose.position.y = 0.4;
    }
    tracking_point.pose.position.z = object_pose.position.z + 0.1; 
    orientation.setRPY(0, tau/4, 0); // tau = 2*pi;
    tracking_point.pose.orientation = tf2::toMsg(orientation);
    ROS_INFO("Tracking point x = %f; y = %f", tracking_point.pose.position.x, tracking_point.pose.position.y);
    pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, tracking_point, tolerance_pose, tolerance_angle);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);
    moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
    req.max_velocity_scaling_factor = 0.7;

    ROS_INFO("Generate path.");
    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
    // representation while planning
    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
      // Now, call the pipeline and check whether planning was successful.
      planning_pipeline->generatePlan(lscene, req, res);
    }

    // Now, call the pipeline and check whether planning was successful.
    // Check that the planning was successful
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    // Visualize the trajectory
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    //visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), 
    joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
    visual_tools.trigger();

    // Execute the plan, set the state in the planning scene to the final state of the last plan
    move_group_interface.execute(response.trajectory);
    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    
    // ------------------------------------------------------------------//
    //-----------Plan a sequence of paths
    // ------------------------------------------------------------------//
    // 
    planning_interface::MotionPlanRequest first_req;
    planning_interface::MotionPlanRequest followed_req;
    moveit_msgs::MotionSequenceRequest seq_req;
    moveit_msgs::MotionSequenceResponse seq_res;
    moveit_msgs::MotionSequenceItem sequence_item;
    moveit_msgs::RobotTrajectory sequence_traj;

    // Initialize first planning request
    // Only this request has a start_state
    first_req.group_name = PLANNING_GROUP;
    first_req.planner_id = "LIN";
    first_req.max_acceleration_scaling_factor = 1;
    first_req.max_velocity_scaling_factor = working_conveyor_vel/max_lin_vel;

    // and second request
    followed_req = first_req;

    // First waypoint: follow and synchronize 
    ROS_INFO("Construct start picking goal");
    geometry_msgs::PoseStamped next_pose;
    next_pose = tracking_point;
    next_pose.pose.position.y -= 0.05;
    pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
    first_req.goal_constraints.clear();
    first_req.goal_constraints.push_back(pose_goal);
    moveit::core::robotStateToRobotStateMsg(*robot_state, first_req.start_state);
    // Add to sequence item vector
    sequence_item.req = first_req;
    sequence_item.blend_radius = 0; // in meter
    seq_req.items.push_back(sequence_item);

    // Next waypoint: down and grasp
    ROS_INFO("Construct down and grasp goal");
    double down_distance = 0.1;
    next_pose.pose.position.z -= down_distance;
    double t_vel_z = down_distance/down_speed; // not including acceleration factor
    next_pose.pose.position.y -= t_vel_z * working_conveyor_vel;
    pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
    followed_req.goal_constraints.clear();
    followed_req.goal_constraints.push_back(pose_goal);
    followed_req.max_velocity_scaling_factor = 
      sqrt(working_conveyor_vel * working_conveyor_vel + down_speed * down_speed)/max_lin_vel;
    // Add to sequence item vector
    sequence_item.req = followed_req;
    sequence_item.blend_radius = 0;
    seq_req.items.push_back(sequence_item);

    // Next waypoint: keep the height of gripper for a while
    ROS_INFO("Construct end of tracking goal");
    next_pose.pose.position.y -= gripping_time * working_conveyor_vel;
    pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
    followed_req.goal_constraints.clear();
    followed_req.goal_constraints.push_back(pose_goal);
    followed_req.max_velocity_scaling_factor = working_conveyor_vel/max_lin_vel;
    // Add to sequence item vector
    sequence_item.req = followed_req;
    sequence_item.blend_radius = 0;
    seq_req.items.push_back(sequence_item);

    // Next waypoint: Up with high speed
    ROS_INFO("Construct picking up goal");
    next_pose.pose.position.z += down_distance; //
    pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
    followed_req.goal_constraints.clear();
    followed_req.goal_constraints.push_back(pose_goal);
    followed_req.max_velocity_scaling_factor = pick_up_speed/max_lin_vel;
    // Add to sequence item vector
    sequence_item.req = followed_req;
    sequence_item.blend_radius = 0;
    seq_req.items.push_back(sequence_item);

    // Next waypoint - to the bin
    ROS_INFO("Construct end goal");
    

    //----------- Chuong-new-OPEN -----------//
    switch(object_pose.type.number ){
      case 0:
        next_pose.pose.position.x = 0.1; // default = 0.85
        next_pose.pose.position.y = 0.2; // +-0.22 for left and right bin
        break;
      case 1:
        next_pose.pose.position.x = 0.1; // default = 0.85
        next_pose.pose.position.y = -0.2; // +-0.22 for left and right bin
        break;
      case 2:
        next_pose.pose.position.x = 0.05; // default = 0.85
        next_pose.pose.position.y = 0.4; // +-0.22 for left and right bin
        break;
      case 3:
        next_pose.pose.position.x = 0.05; // default = 0.85
        next_pose.pose.position.y = -0.4; // +-0.22 for left and right bin
        break;
    }
  
    //----------- Chuong-new-CLOSE -----------//

    //next_pose.pose.position.z += 0.1;
    //orientation.setRPY(0, 0, 0); // tau = 2*pi;
    //next_pose.pose.orientation = tf2::toMsg(orientation);
    pose_goal = kinematic_constraints::constructGoalConstraints(TOOL_LINK, next_pose, tolerance_pose, tolerance_angle);
    followed_req.goal_constraints.clear();
    followed_req.goal_constraints.push_back(pose_goal);
    followed_req.planner_id = "PTP";
    followed_req.max_velocity_scaling_factor = 0.7;
    // Add to sequence item vector
    sequence_item.req = followed_req;
    sequence_item.blend_radius = 0;
    seq_req.items.push_back(sequence_item);

    // ------------------------------------------------------------------//
    //----------- Request sequence of multiple segments
    // ------------------------------------------------------------------//

    // Assign client request
    planner_srv.request.request = seq_req;

    //
    if (_planner_client.call(planner_srv))
    {
      sequence_traj = planner_srv.response.response.planned_trajectories.back();
      ROS_INFO("Planning time: %f s", planner_srv.response.response.planning_time);
      visual_tools.publishTrajectoryLine(sequence_traj, 
        joint_model_group->getLinkModel(TOOL_LINK),joint_model_group);
      visual_tools.trigger();
      ROS_INFO("Plan successfully. Ready for execute");
    }
    else
    {
      ROS_ERROR("Failed to call service /plan_sequence_path");
    }

    // ------------------------------------------------------------------------//
    // Wait for the object to come to tracking point for executing -  temporary solution
    // ------------------------------------------------------------------------//

    // There's a delay when execute a planned tracjectory, so we need to execute with an offset
    double offset_tracking_point = 0.11; // default = 0.04
    double actual_tracking_point = tracking_point.pose.position.y + offset_tracking_point;

    //
    while(object_pose.position.y > actual_tracking_point || 
    object_pose.position.y < (tracking_point.pose.position.y - 0.01))
    {
      ros::Rate(100).sleep();
    }

    ROS_INFO("Tracking start at y = %f", object_pose.position.y);

    // RAPID: SWITCH THE GRIPPER ON HERE

    // Execute the sequence and reset joint states
    double exe_time = ros::Time::now().toSec();
    move_group_interface.execute(sequence_traj);
    exe_time = ros::Time::now().toSec() - exe_time;
    ROS_INFO("Execution time = %fs", exe_time);

    // RAPID: SWITCH THE GRIPPER OFF HERE....

    // Update the robot state
    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(planner_srv.response.response.sequence_start);
    robot_state->setJointGroupPositions(joint_model_group, 
    planner_srv.response.response.planned_trajectories.back().joint_trajectory.points.back().positions);

    visual_tools.deleteAllMarkers();

    // 
    robot_done.publish(done);

    // Make sure the old object has been removed
    while(object_pose.position.y < (0.4 - 0.02))
    {
      ros::Duration(0.1).sleep();
    }
  }
  
  return 0;
}