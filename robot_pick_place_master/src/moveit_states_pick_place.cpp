#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <iostream>
#include <string>

static const size_t capacity = 7;
static const std::string states[capacity] = {"onTopTarget","pickingTarget","targetPicked", "onTopPlacing","placingTarget","afterPlacing","startZeros"};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  //group.setRandomTarget();
  for(size_t i=0;i<sizeof(states);i++){
  		group.setStartStateToCurrentState();//here the robot must have this state as one of its predefined states.
  		group.setNamedTarget(states[i]);
  		group.move();
  		ros::Duration(1.0).sleep();
  		ROS_INFO("The robot is in state %s", states[i].c_str());  	
  }
  
}