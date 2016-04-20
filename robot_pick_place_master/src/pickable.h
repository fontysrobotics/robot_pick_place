#ifndef PICKABLE_H
#define PICKABLE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <string>
#include <iostream>
#include <math.h>
#include <typeinfo>

///// This class let any user create a Pickable object, the user needs to provide the position(x,y,z) and the yawAngle (radians).
///// Please, take into account that the Pickable class needs an asynchronous instance of the ROS node (ros::NodeHandle nh) that runs 
///// the computer vision application, it needs to be shared in construction time.
class Pickable{
    public:
        ///// Constructor of the class Pickable, it is used in the publish_pickable_pose_server to create a single pickable object with only the yaw angle.
        Pickable(const double x_position, const double y_position, const double z_position, const double yaw_angle);
        
        ///// Constructor of the class Pickable, is used in the publish_pickable_pose_server and it shuold be used only if the user wants to create several Pickable objects (an array or vector of Pickable)
        Pickable(const double x_position, const double y_position, const double z_position, const double yaw_angle, const std::string tf_link);
        
        ///// Constructor of the class Pickable, it is used to create a single pickable object with only the yaw angle.
        Pickable(const ros::NodeHandle nh, const double x_position, const double y_position, const double z_position, const double yaw_angle);
        
        ///// Constructor of the class Pickable, it shuold be used only if the user wants to create several Pickable objects (an array or vector of Pickable)
        ///// Where the std::string _tfLink parameter must be provided and different for each Pickable object created.
                ////// ..... NOTE: To name the Pickable objects is recommended to follow this convention: "pickable_link00", "pickable_link01",... "pickable_link99"
        Pickable(const ros::NodeHandle nh, const double x_position, const double y_position, const double z_position, const double yaw_angle, const std::string tf_link);
        
        ///// Constructor of the class Pickable, it is used to create a single pickable object with all the orientation angles (row, pitch, yaw).
        //Pickable(const ros::NodeHandle nh, const double x_position, const double y_position, const double z_position, const double row_angle, const double pitch_angle, const double yaw_angle);
        ///// Constructor of the class Pickable, it shuold be used only if the user wants to create several Pickable objects (an array or vector of Pickable)
        ///// and the user provides all the orientation angles (row, pitch, yaw).
        //Pickable(const ros::NodeHandle nh, const double x_position, const double y_position, const double z_position, const double row_angle, const double pitch_angle, const double yaw_angle, const std::string tf_link);
        
        ///// Constructor of the class Pickable, it is NOT RECOMENDED to use this constructor, it is meant to provide flexibility to the developers, so they
        ///// can create an instance of a Pickable class and assign the values when needed.
        //Pickable(const ros::NodeHandle nh);
        
        ///// Destructor of the class Pickable, it takes care of freeing the memory used by the instances created from the Pickable class.
    	~Pickable();
    	
    	///// Returns a double with the position of the Pickable object in the cartesian coordinate x.
    	double getXPosition();
    	
    	///// Store the position in the cartesian coordinate x of the Pickable object. 
    	void setXPosition(const double x_position);
    	
    	///// Returns a double with the position of the Pickable object in the cartesian coordinate y.
    	double getYPosition();
    	
    	///// Store the position in the cartesian coordinate y of the Pickable object. 
    	void setYPosition(const double y_position);
    	
    	///// Returns a double with the position of the Pickable object in the cartesian coordinate z.
    	double getZPosition();
    	
    	///// Store the position in the cartesian coordinate z of the Pickable object. 
    	void setZPosition(const double z_position);
    	
    	///// Returns a double value of the yaw angle in radians of the pickable object
    	double getRowAngle();
    	
    	///// The _yawAngle MUST BE a double value in radians >=0.
    	void setRowAngle(const double row_angle);
    	
    	///// Returns a double value of the yaw angle in radians of the pickable object
    	double getPitchAngle();
    	
    	///// The _yawAngle MUST BE a double value in radians >=0.
    	void setPitchAngle(const double pitch_angle);
    	
    	///// Returns a double value of the yaw angle in radians of the pickable object
    	double getYawAngle();
    	
    	///// The _yawAngle MUST BE a double value in radians >=0.
    	void setYawAngle(const double yaw_angle);
    	
    	///// Returns the std::string with the 
    	std::string getLink();
    	
    	///// The _tfLink contains a std::string that follows this format: "pickable_linkXX", 
    	///// where XX is the number from 00 to 99, robots will pick the "pickable_link00" first.
    	void setLink(std::string tf_link);
    	
    	///// The user must avoid using this method, it should be used only when the Pickable object 
    	///// was created only with the node hadler parameter, so the user must provide the position and orientation
    	///// of the pickable object.
    	void publish();
    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        ros::Time current_time_;
        ros::Time last_time_;
        tf::TransformBroadcaster *tf_broadcaster_;
        double x_position_;
        double y_position_;
        double z_position_;
        double row_angle_;
        double pitch_angle_;
        double yaw_angle_;
        std::string tf_link_;
        //tf::Quaternion tfQuat;
        geometry_msgs::Quaternion geometry_msgs_quat_;
        geometry_msgs::TransformStamped geometry_msgs_trans_;
        nav_msgs::Odometry odom_;
};

#endif
/*TODO: apply metaprogramming (templates) if possible.*/