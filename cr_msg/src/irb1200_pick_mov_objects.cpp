/* Author: Phat Do */
/* Description:  
Robot: ABB IRB1200 7/70
Planner: Pilz industrial motion planner
Function: Pick a moving object requested from irb1200_objects_publisher client
*/

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group/move_group_capability.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/MoveGroupSequenceAction.h>
#include <moveit_msgs/GetMotionSequence.h>

#include <actionlib/server/simple_action_server.h>


class PickMovingObject
{
  protected:

  public:

  // ros::NodeHandle node_handle;

  // // Define planning group, base frame and end effector link
  // const std::string PLANNING_GROUP = "manipulator";
  // const std::string BASE_FRAME = "base_link";
  // const std::string EE_BASE_LINK = "link_6"; // The link end-effector will be attached on
  // const std::string TOOL_LINK = "tool0"; // Working point

  // // Load the robot description on ROS parameter server
  // robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
  
  // // Construct a robot model, which contains the robot's kinematic info
  // moveit::core::RobotModelPtr robot_model;// = robot_model_loader->getModel();

  // // Construct a planning scene from the robot model loader
  // planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  // // 
  // moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  
};


int main(int argc, char** argv)
{
  // --------------------------------------------------------------------------------------//
  // ROS setup
  // --------------------------------------------------------------------------------------//

  // Initialize the node
  ros::init(argc, argv, "irb1200_pick_moving_object");

  // ROS spinning 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  return 0;
}