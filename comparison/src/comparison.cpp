/************************************************
Comparison file code
This file which operate comparison between
CAO and Num represents one node of the entire
demonstrator
************************************************/

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>

#include <fanuc_grinding_comparison/ComparisonService.h> // Description of the Service we will use

boost::shared_ptr<ros::NodeHandle> node;

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Alway true
 */
bool comparison(fanuc_grinding_comparison::ComparisonService::Request &req,
                fanuc_grinding_comparison::ComparisonService::Response &res)
{
  // Get parameters from the message and print them
  ROS_WARN_STREAM(std::endl << req);

  res.ReturnStatus = true;
  res.ReturnMessage = "comparison";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "comparison");
  node.reset(new ros::NodeHandle);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("comparison_service", comparison);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  spinner.stop();
  return 0;
}
