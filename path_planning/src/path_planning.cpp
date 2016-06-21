/************************************************
Path planner using bezier library.
This file which operate path planning represents
one node of the entire demonstrator
************************************************/

// Standard headers
#include <string>
#include <iostream>

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

#include <path_planning/PathPlanningService.h> // Description of the Service we will use

#include "bezier_library/bezier_library.hpp"

boost::shared_ptr<move_group_interface::MoveGroup> group;
boost::shared_ptr<ros::NodeHandle> node;

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
bool moveRobotPathPlanning(path_planning::PathPlanningService::Request &req,
                           path_planning::PathPlanningService::Response &res)
{
  ROS_WARN_STREAM(std::endl << req);
  std_msgs::String status;
  status.data = "Loading mesh/cloud files";
  status_pub->publish(status);

  std::string package = "path_planning";
  //Get package path
  std::string package_path = ros::package::getPath(package);
  std::string meshes_path = package_path + "/meshes/";
  std::string mesh_ressource = "package://" + package + "/meshes/";
  std::string mesh_ressource_file = "file://";

  // Create publishers for point clouds and markers
  ros::Publisher
  trajectory_publisher,
  input_mesh_publisher,
  defect_mesh_publisher,
  dilated_mesh_publisher,
  normal_publisher;
  trajectory_publisher    = node->advertise<visualization_msgs::Marker>("trajectory", 1);
  input_mesh_publisher    = node->advertise<visualization_msgs::Marker>("input_mesh", 1);
  defect_mesh_publisher   = node->advertise<visualization_msgs::Marker>("defect_mesh", 1);
  dilated_mesh_publisher  = node->advertise<visualization_msgs::Marker>("dilated_mesh", 1);
  normal_publisher        = node->advertise<visualization_msgs::MarkerArray>("normals", 1);

  // Get PLY file name from command line
  std::string input_mesh_filename = req.CADFileName;
  std::string defect_mesh_filename = req.ScanFileName;

  // Determine lean angle axis
  std::string lean_angle_axis;
  if (req.AngleX == true)
    lean_angle_axis = "x";
  else if (req.AngleY == true)
    lean_angle_axis = "y";
  else if (req.AngleZ == true)
    lean_angle_axis = "z";
  else
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Please select a lean angle axis for the effector";
    return true;
  }

  req.DepthOfPath = req.DepthOfPath / 1000; // Bezier needs DepthOfPath in millimeters
  req.GrindDiameter = req.GrindDiameter / 1000; // Bezier needs GrindDiameter in millimeters
  req.CoveringPercentage = req.CoveringPercentage / 100; // Bezier needs CoveringPercentage like a percentage

  std::string rviz_fixed_frame = "base_link";
  std::string rviz_topic_name = normal_publisher.getTopic();

  status.data = "Create Bezier object";
  status_pub->publish(status);
  Bezier bezier_planner(input_mesh_filename,
                        defect_mesh_filename,
                        rviz_fixed_frame,
                        rviz_topic_name,
                        lean_angle_axis,
                        req.AngleValue,
                        req.DepthOfPath,
                        req.GrindDiameter,
                        req.CoveringPercentage,
                        req.ExtricationCoefficient,
                        req.ExtricationFrequency,
                        req.SurfacingMode);
  if(req.Compute)
  {
    status.data = "Compute Bezier trajectory";
    status_pub->publish(status);

    std::vector<bool> points_color_viz;
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
    std::vector<int> index_vector;
    //std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector_pass; // Get pose in this pass
    std::vector<bool> points_color_viz_pass; // Get bool data in this pass (real/extrication path in this pass)

    // Display in RVIZ
    bezier_planner.displayMesh(input_mesh_publisher, mesh_ressource_file + input_mesh_filename);
    bezier_planner.displayMesh(defect_mesh_publisher, mesh_ressource_file + defect_mesh_filename, 0.1, 0.1, 0.1, 0.6);
    bezier_planner.generateTrajectory(way_points_vector, points_color_viz_pass, index_vector);

    // Save dilated meshes
    status.data = "Saving dilated meshes";
    status_pub->publish(status);
    bezier_planner.saveDilatedMeshes(meshes_path + "dilatedMeshes");

    // Copy the vector of Eigen poses into a vector of ROS poses
    res.RobotPosesOutput.clear();
    for(unsigned j = 0; j < way_points_vector.size(); ++j)
    {
      geometry_msgs::Pose tmp;
      tf::poseEigenToMsg(way_points_vector[j], tmp);
      res.RobotPosesOutput.push_back(tmp);
    }

    res.PointColorVizOutput.clear();
    for (unsigned i = 0; i < points_color_viz_pass.size(); ++i)
    {
      res.PointColorVizOutput.push_back(points_color_viz_pass[i]);
    }

    for(unsigned int k = 0; k < index_vector.size(); ++k)
    {
      res.IndexVectorOutput.push_back(index_vector[k]);
    }

    res.ReturnStatus = true;
    res.ReturnMessage = "Trajectory generated";
    return true;
  }
  else
  {
    status.data = "Displaying trajectory";
    status_pub->publish(status);
    // We keep the same trajectory and we send trajectory request in trajectory response
    for(unsigned i = 0; i < req.RobotPosesInput.size(); ++i)
    {
      res.RobotPosesOutput.push_back(req.RobotPosesInput[i]);
      res.PointColorVizOutput.push_back(req.PointColorVizInput[i]);
    }
    for(unsigned j = 0; j < req.IndexVectorInput.size(); ++j)
    {
      res.IndexVectorOutput.push_back(req.IndexVectorInput[j]);
    }
  }

  std::vector<int> index_vector_req;
  // Declare variable which store pose msg given by request
  std::vector<geometry_msgs::Pose> way_points_msg_req;
  // Declare variable which store bool datas (real/extrication path) given by request
  std::vector<bool> points_color_viz_req; // Get bool data in this pass (real/extrication path in this pass)
  std::string number;

  tf::TransformListener listener;
  listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.0));
  for (unsigned int k = 0; k < req.RobotPosesInput.size() - 1; ++k)
  {
    way_points_msg_req.push_back(req.RobotPosesInput[k]);
    points_color_viz_req.push_back(req.PointColorVizInput[k]);
  }
  for (unsigned int p = 0; p < req.IndexVectorInput.size(); ++p)
  {
    index_vector_req.push_back(req.IndexVectorInput[p]);
  }

  for (unsigned int i = 0; i < index_vector_req.size() - 1; i++)
  { //For each pass
    // Create a vector to store pose messages for each pass ( we use it in order to execute trajectory )
    std::vector<geometry_msgs::Pose> way_points_msg_pass;
    way_points_msg_pass.insert(way_points_msg_pass.begin(), way_points_msg_req.begin() + index_vector_req[i] + 1,
                                   way_points_msg_req.begin() + index_vector_req[i + 1]);
    std::vector<bool> points_color_viz_pass; // Get bool data in this pass (real/extrication path in this pass)
    points_color_viz_pass.insert(points_color_viz_pass.begin(), points_color_viz_req.begin() + index_vector_req[i] + 1,
                                 points_color_viz_req.begin() + index_vector_req[i + 1]);
    number = boost::lexical_cast<std::string>(i);

    // Create a matrix of poses for each pass ( we use it in order to visualize trajectory )
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector_pass;
    way_points_vector_pass.resize(way_points_msg_pass.size());
    for (size_t h = 0; h < way_points_msg_pass.size(); h++)
    {
      tf::poseMsgToEigen(way_points_msg_pass[h], way_points_vector_pass[h]);
      tf::Transform world_to_link6_tf, world_to_tcp_tf;
      geometry_msgs::Pose world_to_link6 = way_points_msg_pass[h];
      geometry_msgs::Pose world_to_tcp;
      tf::poseMsgToTF(world_to_link6, world_to_link6_tf);
      world_to_tcp_tf = world_to_link6_tf;
      tf::poseTFToMsg(world_to_tcp_tf, world_to_tcp);
      way_points_msg_pass[h] = world_to_tcp;
    }

    if (req.Visualization == true || req.Simulation == true)
    {
      // Visualize this trajectory
      bezier_planner.displayMesh(dilated_mesh_publisher, mesh_ressource + "dilatedMeshes/mesh_" + number + ".ply");
      bezier_planner.displayTrajectory(way_points_vector_pass, points_color_viz_pass, trajectory_publisher);
      //bezier_planner.displayNormal(way_points_vector_pass, points_color_viz_pass, normal_publisher); // Display normals in this pass

      if (req.Simulation == false)
        sleep(2);
      else
      {
        // Execute this trajectory
        moveit_msgs::ExecuteKnownTrajectory srv;
        srv.request.wait_for_execution = true;
        ros::ServiceClient executeKnownTrajectoryServiceClient =
            node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
        group->computeCartesianPath(way_points_msg_pass, 0.05, 0.0, srv.request.trajectory);
        executeKnownTrajectoryServiceClient.call(srv);
      }
    }
    else
    {
      res.ReturnStatus = false;
      res.ReturnMessage = "Internal problem between Visualization and Simulation mode... Aborted!";
      return true;
    }
  }

  res.ReturnStatus = true;
  if (req.Simulation == true)
    res.ReturnMessage = "Simulation succeeded";
  else
    res.ReturnMessage = "Visualization succeeded";

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_planning");
  node.reset(new ros::NodeHandle);

  status_pub.reset(new ros::Publisher);
  *status_pub = node->advertise<std_msgs::String>("scanning_status", 1);

  // Initialize move group
  group.reset(new move_group_interface::MoveGroup("grinding_disk"));
  group->setPoseReferenceFrame("/base_link");
  group->setPlannerId("RRTConnectkConfigDefault");
  group->setPlanningTime(2);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("path_planning_service", moveRobotPathPlanning);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  spinner.stop();
  return 0;
}
