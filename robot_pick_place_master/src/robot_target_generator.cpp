#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "robot_pick_place/GenerateRobotTargets.h"


geometry_msgs::PoseStamped pose_msg;//here it should be a pose message and not poseStamped
bool _is_generated = false;

////// Here the transform listener is crerated to ensure that the transform to pickable_link00 exist, if it does, then it will make the robot move. 

bool generate(robot_pick_place::GenerateRobotTargets::Request  &req, robot_pick_place::GenerateRobotTargets::Response &res){
    res.poses.push_back(pose_msg);
    res.is_generated = _is_generated;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_target_generator");

    ros::NodeHandle node;
  
    //std::vector<geometry_msgs::PoseStamped::ConstPtr> poses;
    
    ros::ServiceServer service = node.advertiseService("generate_robot_targets", generate);

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (node.ok()){
        tf::StampedTransform transform;
         _is_generated = false;
        try{
            // Here we not only wait, we also keep track of the transform and we decided if the object is moving or if it is static based on the presettings
            // the user can specify if there are moving objects.
            // To control the object future location, then a vector of sample locations is created to have a calculated speed in x, y and z. 
            if(listener.waitForTransform("/pickable_link00", "/base_link", ros::Time(0), ros::Duration(10.0) )){
                listener.lookupTransform("/pickable_link00", "/base_link", ros::Time(0), transform);
                ROS_INFO("The transform exist");
                // Here the transform needs to be converted to a pose message, you have to do it one by one, not use any function.
                
                tf::Stamped<tf::Pose> grasp_tf_pose(transform);
                tf::poseStampedTFToMsg(grasp_tf_pose, pose_msg);
                _is_generated = true;
            }
            else{
                ROS_INFO("The transform is not available");
            }
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        
        
        rate.sleep();
    }
    return 0;
};

//To make the robot move, then use two parameters, one the bool returned by the service, and the other a counter for the number of try.