/************************************************
Post processor using fanuc_post_processor_library.
This file which operate post processor represents
one node of the entire demonstrator
************************************************/

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Transform.h>
#include "fanuc_post_processor_library/fanuc_post_processor_library.hpp"

#include <fanuc_grinding_post_processor/PostProcessorService.h> // Description of the Service we will use

boost::shared_ptr<ros::NodeHandle> node;

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Always true
 */
bool postProcessor(fanuc_grinding_post_processor::PostProcessorService::Request &req,
                   fanuc_grinding_post_processor::PostProcessorService::Response &res)
{
  // Get parameters from the message and print them
  //ROS_INFO_STREAM(std::endl << req);

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > robot_poses_eigen;

  for (geometry_msgs::Pose &tmp : req.RobotPoses)
  {
    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(tmp, pose);
    pose.translation() += Eigen::Vector3d(0, 0, req.TrajectoryZOffset);
    robot_poses_eigen.push_back(pose);
  }

  if (robot_poses_eigen.empty())
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Trajectory is empty";
    return true;
  }

  if (robot_poses_eigen.size() != req.IsGrindingPose.size())
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Trajectory is not correct";
    return true;
  }

  FanucPostProcessor fanuc_pp;
  fanuc_pp.setProgramName(req.ProgramName);
  fanuc_pp.setProgramComment(req.Comment);

  unsigned int speed;
  const double grinding_disk_DO(3);

  // First pose is a machining pose, we have to switch on the DO
  fanuc_pp.appendDigitalOutput(grinding_disk_DO, true);
  fanuc_pp.appendWait(1);
  //speed = req.MachiningSpeed * 100;
  speed = 30;
  fanuc_pp.appendPoseCNT(FanucPostProcessor::JOINT, robot_poses_eigen[0], 1, speed, FanucPostProcessor::PERCENTAGE, 100);

  for (unsigned i = 1; i < robot_poses_eigen.size(); ++i)
  {
    // if the new point is an extrication point and the old one was a machining point, we have to switch off the DO
    if (req.IsGrindingPose[i] == 0 && req.IsGrindingPose[i - 1] == 1)
    {
      fanuc_pp.appendDigitalOutput(grinding_disk_DO, false);
      fanuc_pp.appendWait(1);
      speed = 50;//req.ExtricationSpeed * 100;
    }
    // if the new point is an machining point and the old one was a extrication point, we have to switch on the DO
    else if (req.IsGrindingPose[i] == 1 && req.IsGrindingPose[i - 1] == 0)
    {
      fanuc_pp.appendDigitalOutput(grinding_disk_DO, true);
      fanuc_pp.appendWait(1);
      speed = 30;//req.MachiningSpeed * 100;
    }
    fanuc_pp.appendPoseCNT(FanucPostProcessor::JOINT, robot_poses_eigen[i], i + 1, speed, FanucPostProcessor::PERCENTAGE, 100);
  }

  std::string program;
  fanuc_pp.generateProgram(program);
  //ROS_INFO_STREAM("This is the generated program:\n\n" << program);

  // TODO: Check if writing in a .ls file works
  std::ofstream tp_program_file;
  std::string file_location = req.ProgramLocation + req.ProgramName;


  tp_program_file.open(file_location.c_str(), std::ios::out);
  // We check if file have been opened correctly
  if (tp_program_file.bad())
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Cannot open/create TP program file";
    return true;
  }

  tp_program_file << program; // We put program into TP program file
  tp_program_file.close();

  if (!req.Upload)
  {
    res.ReturnStatus = true;
    res.ReturnMessage = "Program generated";
    return true;
  }

  // TODO: Try to upload
  ROS_INFO_STREAM("Try to upload program on IP address: " << req.IpAdress);
  if (!fanuc_pp.uploadToFtp(req.IpAdress))
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Cannot upload the program on IP address: " + req.IpAdress;
    return true;
  }

  res.ReturnStatus = true;
  res.ReturnMessage = "Program generated and uploaded";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "post_processor");
  node.reset(new ros::NodeHandle);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("post_processor_service", postProcessor);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }

  spinner.stop();
  return 0;
}
