#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "pickable.h"
#include "robot_pick_place/PublishPickablePose.h"// Here the service is converted in running time to any programing lenguage that might be using it, for c++ it generates a .h file.



bool pub(robot_pick_place::PublishPickablePose::Request  &req, robot_pick_place::PublishPickablePose::Response &res){
    //res.sum = req.a + req.b;
    if(req.tf_link.size() > 0){
        Pickable *pick = new Pickable(req.x_position,req.y_position,req.z_position,req.yaw_angle,req.tf_link);
        ROS_INFO("The pickable is created with the tf_link");
    }   
    else{
        Pickable *pick = new Pickable(req.x_position,req.y_position,req.z_position,req.yaw_angle);
        ROS_INFO("The pickable is created without the tf_link");
    }
    
    //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pickable_pose_publisher_server");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("publish_pickable_pose", pub);
    ROS_INFO("Ready to publish the pickable pose.");
    ros::spin();
    
    return 0;
}