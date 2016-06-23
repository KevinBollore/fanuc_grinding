/************************************************
Post processor using fanuc_post_processor_library.
This file which operate post processor represents
one node of the entire demonstrator
************************************************/

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
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
  ROS_WARN_STREAM(std::endl << req);

  FanucPostProcessor fanuc_pp;
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

  std::string program;
  fanuc_pp.generateProgram(program);
  ROS_WARN_STREAM("This is the generated program:\n\n" << program);

  // TODO: Check if writing in a .ls file works
  std::ofstream tp_program_file;
  std::string file_location = req.ProgramLocation + req.ProgramName;


  tp_program_file.open(file_location.c_str(), std::ios::out);
  // We check if file have been opened correctly
  if(tp_program_file.bad())
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
