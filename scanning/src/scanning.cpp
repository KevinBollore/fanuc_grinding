/************************************************
Scanner
This file which operate scanning represents
one node of the entire demonstrator
************************************************/

// Standard headers
#include <string>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/String.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/mark.h>

#include <scanning/ScanningService.h> // Description of the Service we will use
#include <execute_joint_state/ExecuteJointStateService.h> // We will call this service
#include <publish_meshfile/PublishMeshfileService.h>

#include "yaml_utils.h"

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/io/davidsdk_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>

// PCL davidSDK object pointer
pcl::DavidSDKGrabber::Ptr davidsdk_ptr;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

boost::shared_ptr<ros::NodeHandle> node;

/** Status publisher */
boost::shared_ptr<ros::Publisher> status_pub;

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Always true
 */
bool moveRobotScan(scanning::ScanningService::Request &req, scanning::ScanningService::Response &res)
{
  std_msgs::String status;
  status.data = "Parsing YAML trajectory file";
  status_pub->publish(status);

  // Parse YAML File
  const std::string filename= req.YamlFileName;
  std::string reference_frame;
  std::string tool_reference;
  std::vector<double> joint;
  std::vector<std::vector<double> > joint_list;
  try
  {
    YAML::Node yamlFile;
    yaml_parser::yamlNodeFromFileName(filename, yamlFile);

    // Parse "reference" node in YAML file
    // Just say no node if no node -->exception
    const YAML::Node & reference_node = yaml_parser::parseNode(yamlFile, "reference");
    if(!yamlFile["reference"])
    {
      YAML::Mark mark;
      YAML::Exception yaml_exception(mark, "Cannot parse 'reference' node");
      throw yaml_exception;
    }

    // Parse "reference_frame" tag in "reference" node in YAML file
    yaml_parser::parseString(reference_node, "reference_frame", reference_frame);

    // Parse "tool" node in YAML file
    const YAML::Node & tool_node = yaml_parser::parseNode(yamlFile, "tool");
    if(!yamlFile["tool"])
    {
      YAML::Mark mark;
      YAML::Exception yaml_exception(mark,"Cannot parse 'tool' node");
      throw yaml_exception;
    }

    // Parse "tool_reference" tag in "tool" node in YAML file
    yaml_parser::parseString(tool_node, "tool_reference", tool_reference);

    // Parse "joint_values" node in YAML file
    const YAML::Node & joint_node = yaml_parser::parseNode(yamlFile, "joint_values");
    if(!yamlFile["joint_values"])
    {
      YAML::Mark mark;
      YAML::Exception yaml_exception(mark,"Cannot parse 'joint_values' node");
      throw yaml_exception;
    }

    // Parse all "joint" tag in "joint_values" node in YAML file
    for(unsigned j = 0; j < joint_node.size(); ++ j)
    {
      yaml_parser::parseVectorD(joint_node[j], "joint", joint);
      joint_list.push_back(joint);
    }
  }
  catch(const YAML::Exception &e)
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Problem occurred during parsing yaml file for joint values, problem is: " + e.msg;
    return true;
  }

  // Parse YAML file in order to obtain sls_2_calibration matrix
  status.data = "Parsing YAML calibration file";
  status_pub->publish(status);
  const std::string filenameCalibration= req.YamlCalibrationFileName;
  std::vector<double> n_vector;
  std::vector<double> o_vector;
  std::vector<double> a_vector;
  std::vector<double> p_vector;
  try
  {
    YAML::Node yamlFileCalibration;
    yaml_parser::yamlNodeFromFileName(filenameCalibration, yamlFileCalibration);

    // Parse all tags in "sls_2_calibration_values" node in YAML file
    const YAML::Node& sls_2_node = yaml_parser::parseNode(yamlFileCalibration, "sls_2_calibration_values");
    if(!yamlFileCalibration["sls_2_calibration_values"])
    {
      YAML::Mark mark;
      YAML::Exception yaml_exception(mark,"Cannot parse 'sls_2_calibration_values'");
      throw yaml_exception;
    }
    yaml_parser::parseVectorD(sls_2_node[0], "n", n_vector);
    yaml_parser::parseVectorD(sls_2_node[1], "o", o_vector);
    yaml_parser::parseVectorD(sls_2_node[2], "a", a_vector);
    yaml_parser::parseVectorD(sls_2_node[3], "p", p_vector);
  }
  catch (const YAML::Exception &e)
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Problem parsing YAML calibration file, error:\n" + e.msg;
    return true;
  }

  // We fill the calibration_sls2 transformation matrix with values contained in the YAML file
  Eigen::Affine3d calib_sls_2;
  calib_sls_2.matrix() <<
      n_vector[0], o_vector[0], a_vector[0], p_vector[0],
      n_vector[1], o_vector[1], a_vector[1], p_vector[1],
      n_vector[2], o_vector[2], a_vector[2], p_vector[2],
      n_vector[3], o_vector[3], a_vector[3], p_vector[3];

  // We have to invert this matrix in order to give point cloud in a correct frame
  calib_sls_2 = calib_sls_2.inverse();

  // Declaration of ROS listener
  tf::TransformListener listener;
  // Grab transformation between /sls_2_frame and tool0
  tf::StampedTransform transform_sls_2_tool0;
  Eigen::Affine3d calibration;

  listener.waitForTransform("/base", "/sls_2_frame", ros::Time(0), ros::Duration(1.5));
  listener.lookupTransform("/tool0", "/sls_2_frame", ros::Time(0), transform_sls_2_tool0);
  tf::transformTFToEigen(transform_sls_2_tool0, calibration);

  // Connect SLS-2 Camera
  status.data = "Connecting to SLS-2 server...";
  status_pub->publish(status);
  std::string davidSDK_ip = req.SLS2IpAddress;
  std::string davidSDK_server_name = req.SLS2ServerName;

  // Connect to the SLS_2
  davidsdk_ptr.reset(new pcl::DavidSDKGrabber);
  ROS_INFO_STREAM("Trying to connect to DavidSDK IP \"" << davidSDK_ip << "\" and server name \"" << davidSDK_server_name << "\"");
  davidsdk_ptr->connect(davidSDK_ip);
  if (!davidsdk_ptr->isConnected())
  {
    ROS_ERROR_STREAM("Cannot connect to the David SLS-2 scanner (IP: " << davidSDK_ip << ")");
    ROS_ERROR_STREAM("Server name: " << davidSDK_server_name);
    res.ReturnStatus = false;
    res.ReturnMessage = "Cannot connect to the David SLS-2 scanner";
    return true;
  }
  davidsdk_ptr->setLocalAndRemotePaths("/var/tmp/davidsdk/", "\\\\" + davidSDK_server_name + "\\davidsdk\\");
  davidsdk_ptr->setFileFormatToSTL(); // Best performance

  status.data = "SLS-2 connected";
  status_pub->publish(status);

  // Create entire point cloud
  PointCloudXYZ::Ptr stacked_point_cloud (new PointCloudXYZ());

  // For each joint state, we call a service which execute this joint state
  execute_joint_state::ExecuteJointStateService srv_execute_joint_state;
  ros::ServiceClient execute_joint_state_service =
      node->serviceClient<execute_joint_state::ExecuteJointStateService>("execute_joint_state_service");

  for(unsigned k = 0; k < joint_list.size(); ++k)
  {
    // Add the joint state into the request
    for(unsigned j = 0; j < joint.size(); ++j)
      srv_execute_joint_state.request.JointState[j]=joint_list[k][j];

    execute_joint_state_service.call(srv_execute_joint_state);

    if(srv_execute_joint_state.response.ReturnStatus == false)
    {
      res.ReturnStatus = false;
      res.ReturnMessage = "Problem occurred during scanning.\n" + srv_execute_joint_state.response.ReturnMessage;
      return true;
    }
    else
    {
      // Grab robot transform
      tf::StampedTransform transform;
      sleep(1);
      listener.waitForTransform("/base", "/sls_2_frame", ros::Time::now(), ros::Duration(1.5));
      listener.lookupTransform("/base_link", "/tool0", ros::Time(0), transform);
      Eigen::Affine3d matrix_transform;
      tf::transformTFToEigen(transform, matrix_transform);

      // Grab point cloud
      PointCloudXYZ::Ptr point_cloud (new PointCloudXYZ());
      if (!davidsdk_ptr->grabSingleCloud(*point_cloud))
      {
        res.ReturnStatus = false;
        res.ReturnMessage = "Could not capture point cloud";
        return true;
      }
      status.data = "Point cloud " + boost::lexical_cast<std::string>(k + 1) +
                    "/" + boost::lexical_cast<std::string>(joint_list.size()) + " captured";
      status_pub->publish(status);

      // Transform point cloud in order into the robot frame
      Eigen::Affine3d transformation_pc(Eigen::Affine3d::Identity());
      transformation_pc = matrix_transform * calibration * calib_sls_2;
      transformation_pc *= 0.001; // Transform millimeters to meters
      pcl::transformPointCloud(*point_cloud, *point_cloud, transformation_pc);

      // Store it into global point cloud
      *stacked_point_cloud += *point_cloud;
    }
  }

  // Apply a voxel grid to make entire point cloud homogeneous
  status.data = "Applying voxel grid on point clouds";
  status_pub->publish(status);

  pcl::VoxelGrid<PointXYZ> sor;
  double leaf_size (0.003);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.setInputCloud (stacked_point_cloud);
  sor.filter (*stacked_point_cloud);

  if(stacked_point_cloud->size() == 0)
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Filtered point cloud is empty";
    return true;
  }

  // Save the point cloud with CAD meshname and a suffix
  res.NumerizedMeshName = req.CADName.substr(0, req.CADName.size() - 4) + "_defect.ply";
  pcl::io::savePLYFileBinary(res.NumerizedMeshName, *stacked_point_cloud);

  // Call publish_meshfile service
  publish_meshfile::PublishMeshfileService srv_publish_meshfile;
  ros::ServiceClient publish_meshfile_service =
        node->serviceClient<publish_meshfile::PublishMeshfileService>("publish_meshfile_service");

  // Publish final point cloud
  status.data = "Publishing final point cloud";
  status_pub->publish(status);
  srv_publish_meshfile.request.MeshName = res.NumerizedMeshName;
  srv_publish_meshfile.request.MarkerName = req.MarkerName;
  srv_publish_meshfile.request.PosX =
  srv_publish_meshfile.request.PosY =
  srv_publish_meshfile.request.PosZ =
  srv_publish_meshfile.request.RotX =
  srv_publish_meshfile.request.RotY =
  srv_publish_meshfile.request.RotZ = 0.0;
  srv_publish_meshfile.request.RotW = 1.0;
  publish_meshfile_service.call(srv_publish_meshfile);

  res.ReturnStatus = true;
  boost::filesystem::path p(res.NumerizedMeshName);
  res.ReturnMessage = "Scan succeeded, file:\n" + p.filename().string();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanning");
  node.reset(new ros::NodeHandle);

  status_pub.reset(new ros::Publisher);
  *status_pub = node->advertise<std_msgs::String>("scanning_status", 1);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("scanning_service", moveRobotScan);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  return 0;
}
