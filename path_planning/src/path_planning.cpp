/************************************************
 Path planner using bezier library.
 This file which operate path planning represents
 one node of the entire demonstrator
 ************************************************/

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include "bezier_library/bezier_grinding_surfacing.hpp"

#include <fanuc_grinding_path_planning/PathPlanningService.h> // Description of the Service we will use

boost::shared_ptr<move_group_interface::MoveGroup> group;
boost::shared_ptr<ros::NodeHandle> node;
EigenSTL::vector_Affine3d way_points_vector;

/** Status publisher */
boost::shared_ptr<ros::Publisher> status_pub;

/** Name of the move_group used to move the robot during calibration */
const std::string move_group_name("grinding_disk");
/** Name of the TCP that should be used to compute the trajectories */
const std::string tcp_name("/grinding_disk_tcp");

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Always true
 */
bool pathPlanning(fanuc_grinding_path_planning::PathPlanningService::Request &req,
                  fanuc_grinding_path_planning::PathPlanningService::Response &res)
{
  ROS_INFO_STREAM(std::endl << req);

  if (!req.SurfacingMode)
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Grinding mode is not implemented yet!";
    return true;
  }

  std_msgs::String status;
  std::vector<bool> is_grinding_pose;

  if (req.Compute)
  {
    std::string package = "fanuc_grinding_path_planning";
    //Get package path
    std::string mesh_ressource = "package://" + package + "/meshes/";
    std::string mesh_ressource_file = "file://";

    // Get PLY file name from command line
    std::string input_mesh_filename = req.CADFileName;

    // Determine lean angle axis
    std::string lean_angle_axis;
    BezierGrindingSurfacing::AXIS_OF_ROTATION lean_axis;
    if (req.AngleX == true)
      lean_axis = BezierGrindingSurfacing::X;
    else if (req.AngleY == true)
      lean_axis = BezierGrindingSurfacing::Y;
    else if (req.AngleZ == true)
      lean_axis = BezierGrindingSurfacing::Z;
    else
    {
      res.ReturnStatus = false;
      res.ReturnMessage = "Please select a lean angle axis for the effector";
      return true;
    }

    BezierGrindingSurfacing bezier(input_mesh_filename, req.GrinderWidth, req.CoveringPercentage, req.ExtricationRadius,
                                   req.LeanAngle, lean_axis);

    status.data = "Generate Bezier trajectory";
    status_pub->publish(status);
    std::string error_string;
    error_string = bezier.generateTrajectory(way_points_vector, is_grinding_pose);

    if (!error_string.empty())
    {
      res.ReturnStatus = false;
      res.ReturnMessage = error_string;
      return true;
    }
  }

  // Copy the vector of Eigen poses into a vector of ROS poses
  std::vector<geometry_msgs::Pose> way_points_msg;
  for (Eigen::Affine3d pose : way_points_vector)
  {
    geometry_msgs::Pose tmp;
    tf::poseEigenToMsg(pose, tmp);
    way_points_msg.push_back(tmp);
  }

  res.RobotPosesOutput = way_points_msg;
  for (std::vector<bool>::const_iterator iter(is_grinding_pose.begin()); iter != is_grinding_pose.end(); ++iter)
    res.IsGrindingPose.push_back(*iter);

  if (!req.Simulate)
  {
    res.ReturnStatus = true;
    res.ReturnMessage = boost::lexical_cast<std::string>(way_points_msg.size()) + " poses generated";
    return true;
  }

  tf::TransformListener listener;
  listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.0));

  // Execute this trajectory
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.wait_for_execution = true;
  ros::ServiceClient executeKnownTrajectoryServiceClient = node->serviceClient < moveit_msgs::ExecuteKnownTrajectory
      > ("/execute_kinematic_path");
  double percentage = group->computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory);

  status.data = boost::lexical_cast<std::string>(percentage*100) + "% of the trajectory will be executed";
  status_pub->publish(status);
  executeKnownTrajectoryServiceClient.call(srv);

  if (percentage < 0.9)
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Could not compute the whole trajectory!";
  }
  else
    res.ReturnStatus = true;

  return true;
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "path_planning");
  node.reset(new ros::NodeHandle);

  status_pub.reset(new ros::Publisher);
  *status_pub = node->advertise <std_msgs::String> ("scanning_status", 1);

  // Initialize move group
  group.reset(new move_group_interface::MoveGroup("grinding_disk"));
  group->setPoseReferenceFrame("/base_link");
  group->setPlannerId("RRTConnectkConfigDefault");
  group->setPlanningTime(2);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("path_planning_service", pathPlanning);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  spinner.stop();
  return 0;
}
