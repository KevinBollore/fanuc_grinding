/************************************************
 ExecuteJointState file code
 This file which operate rotation of robot arm
 represents one node of the entire
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

#include <execute_joint_state/ExecuteJointStateService.h> // Description of the Service we will use

boost::shared_ptr<move_group_interface::MoveGroup> group;
boost::shared_ptr<ros::NodeHandle> node;

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Always true
 */
bool moveRobotExecuteJointState(execute_joint_state::ExecuteJointStateService::Request &req,
                                execute_joint_state::ExecuteJointStateService::Response &res)
{
  // We create a joint state and fill it with the request of JointState
  std::vector<double> joint_state;
  for (unsigned i = 0; i < req.JointState.size(); ++i)
  {
    joint_state.push_back(req.JointState[i]);
  }

  // Give the joint state to execute
  group->setJointValueTarget(joint_state);
  // move robot, if there is a problem ( reach axis limit, detect collision ... ),
  // it will reach the nearest joint and will return false
  if (group->move() == false)
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Cannot reach joint values";
    return true;
  }

  res.ReturnStatus = true;
  res.ReturnMessage = "";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_joint_state");
  node.reset(new ros::NodeHandle);

  // Initialize move group
  group.reset(new move_group_interface::MoveGroup("manipulator"));
  group->setPlannerId("RRTConnectkConfigDefault");
  group->setPoseReferenceFrame("/base");
  group->setPlanningTime(2);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("execute_joint_state_service", moveRobotExecuteJointState);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (node->ok())
  {
  }
  return 0;
}
