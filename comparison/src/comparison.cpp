/************************************************
Comparison file code
This file which operate comparison between
CAO and Num represents one node of the entire
demonstrator
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
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <comparison/ComparisonService.h> // Description of the Service we will use

boost::shared_ptr<move_group_interface::MoveGroup> group;
boost::shared_ptr<ros::NodeHandle> node;

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Alway true
 */
bool moveRobotComparison(comparison::ComparisonService::Request &req, comparison::ComparisonService::Response &res)
{
  // Get parameters from the message and print them
  ROS_WARN_STREAM(std::endl << req);
  // Get a param from request: int testParam = req.Param;

  // Get current robot pose
  tf::TransformListener listener;
  listener.waitForTransform("/base", "/tool0", ros::Time::now(), ros::Duration(1.0));
  tf::StampedTransform transform_stamp;
  Eigen::Affine3d current_pose;
  try
  {
    listener.lookupTransform("/base", "/tool0", ros::Time(0), transform_stamp);
    transformTFToEigen(transform_stamp, current_pose);
  }
  catch (tf::TransformException &ex)
  {
    res.ReturnMessage = ex.what();
    res.ReturnStatus = false;
    return true;
  }

  res.ReturnStatus = true;
  res.ReturnMessage = "comparison OK JE RECOIS BIEN";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "comparison");
  node.reset(new ros::NodeHandle);

  // Initialize move group
  group.reset(new move_group_interface::MoveGroup("manipulator"));
  group->setPoseReferenceFrame("/base");
  group->setPlanningTime(2);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("comparison_service", moveRobotComparison);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  return 0;
}
