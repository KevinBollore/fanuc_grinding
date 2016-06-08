/************************************************
Post processor using fanuc_post_processor_library.
This file which operate post processor represents
one node of the entire demonstrator
************************************************/

// Standard headers
#include <string>
#include <iostream>
#include <fstream>

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

#include <post_processor/PostProcessorService.h> // Description of the Service we will use

#include "fanuc_post_processor_library/fanuc_post_processor_library.hpp"

boost::shared_ptr<move_group_interface::MoveGroup> group;
boost::shared_ptr<ros::NodeHandle> node;

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
bool RobotPostProcessor(post_processor::PostProcessorService::Request &req,
                        post_processor::PostProcessorService::Response &res)
{
  // Get parameters from the message and print them
  ROS_WARN_STREAM(std::endl << req);

  std::string package = "post_processor";

  FanucPostProcessor fanuc_pp;

  // Little example
  fanuc_pp.setProgramName(req.ProgramName);
  fanuc_pp.setProgramComment(req.Comment);
  //fanuc_pp.appendDigitalOutput(5, true); // Switch on DO for grinding disk.
  //TODO: Take a look for grinder number... Or create one

  for(unsigned i = 0; i < req.RobotPoses.size(); ++i)
  {
    // TODO: Fix this. We have to know which function of fanuc_post_processor we have to use
    //fanuc_pp.appendPoseCNT(FanucPostProcessor::JOINT, req.RobotPoses[i], 2, 20, FanucPostProcessor::PERCENTAGE, 100);
  }

  //fanuc_pp.appendDigitalOutput(5, false); // Switch off DO for grinding disk.
  fanuc_pp.appendDataMonitorStop();
  fanuc_pp.appendJumpLabel(1);

  std::string program;
  fanuc_pp.generateProgram(program);
  ROS_WARN_STREAM("This is the generated program:\n\n" << program);

  // TODO: Check if writing in a .ls file works
  std::ofstream tp_program_file;
  std::string file_location = req.ProgramLocation;
  // We must convert the request to a std::string and this string to a char*
  tp_program_file.open(file_location.c_str(), std::ios::out);
  // We check if file have been open correctly
  if(tp_program_file.bad())
  {
    // if there is a problem with file opening, we abort this service
    res.ReturnStatus = false;
    res.ReturnMessage = "Cannot open/create TP program file";

    return true;
  }

  tp_program_file << program; // We put program into TP program file

  tp_program_file.close();

  res.ReturnStatus = true;
  res.ReturnMessage = "Post processor program generated";

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "post_processor");
  node.reset(new ros::NodeHandle);

  // Initialize move group
  group.reset(new move_group_interface::MoveGroup("grinding_disk"));
  group->setPoseReferenceFrame("/base_link");
  group->setPlannerId("RRTConnectkConfigDefault");
  group->setPlanningTime(2);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("post_processor_service", RobotPostProcessor);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  return 0;
}
