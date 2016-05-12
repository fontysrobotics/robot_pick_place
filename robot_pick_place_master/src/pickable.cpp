#include "pickable.h"

////------------------ Constructors ------------------------------
Pickable::Pickable(const double x_position, const double y_position, const double z_position, const double yaw_angle){
    tf_broadcaster_ = new tf::TransformBroadcaster();
    setXPosition(x_position);
    setYPosition(y_position);
    setZPosition(z_position);
    setYawAngle(yaw_angle);
    tf_link_ = "pickable_link00";
    publish();
}

Pickable::Pickable(const double x_position, const double y_position, const double z_position, const double yaw_angle, const std::string tf_link){
    tf_broadcaster_ = new tf::TransformBroadcaster();
    setXPosition(x_position);
    setYPosition(y_position);
    setZPosition(z_position);
    setYawAngle(yaw_angle);
    tf_link_ = tf_link;
    publish();
}

////---------------- Destructor -----------------------
Pickable::~Pickable(){
    delete tf_broadcaster_;
}

////---------------- Setters --------------------------
void Pickable::setXPosition(const double x_position){
    Pickable::x_position_ = x_position;
}

void Pickable::setYPosition(const double y_position){
    Pickable::y_position_ = y_position;
}

void Pickable::setZPosition(const double z_position){
    Pickable::z_position_ = z_position;
}

void Pickable::setRollAngle(const double roll_angle){
    if(roll_angle >= 0.0)
        Pickable::roll_angle_ = roll_angle;
    else
        Pickable::roll_angle_ = 0.0;
}

void Pickable::setPitchAngle(const double pitch_angle){
    if(pitch_angle >= 0.0)
        Pickable::pitch_angle_ = pitch_angle;
    else
        Pickable::pitch_angle_ = 0.0;
}

void Pickable::setYawAngle(const double yaw_angle){
    if(yaw_angle >= 0.0)
        Pickable::yaw_angle_ = yaw_angle;
    else
        Pickable::yaw_angle_ = 0.0;
}

void Pickable::setLink(const std::string tf_link){
    Pickable::tf_link_ = tf_link;
}

////---------------- Getters----------------------
double Pickable::getXPosition(){
    return Pickable::x_position_;
}

double Pickable::getYPosition(){
    return Pickable::y_position_;
}

double Pickable::getZPosition(){
    return Pickable::z_position_;
}

double Pickable::getRollAngle(){
    return Pickable::row_angle_;
}

double Pickable::getPitchAngle(){
    return Pickable::pitch_angle_;
}

double Pickable::getYawAngle(){
    return Pickable::yaw_angle_;
}

std::string Pickable::getLink(){
    return Pickable::tf_link_;
}

////------------------- Methods ----------------------
void Pickable::publish(){
    current_time_ = ros::Time::now();
	geometry_msgs_quat_ = tf::createQuaternionMsgFromYaw(Pickable::yaw_angle_);
	geometry_msgs_trans_.header.stamp = current_time_;
	geometry_msgs_trans_.header.frame_id = "base_link";
	geometry_msgs_trans_.child_frame_id = Pickable::tf_link_;//oss.str();
			
	geometry_msgs_trans_.transform.translation.x = Pickable::x_position_;
	geometry_msgs_trans_.transform.translation.y = Pickable::y_position_;
	geometry_msgs_trans_.transform.translation.z = Pickable::z_position_;//10/(shapes[i]->getArea()*0.055555);
	geometry_msgs_trans_.transform.rotation = geometry_msgs_quat_;
    tf_broadcaster_->sendTransform(geometry_msgs_trans_);
    
// 	odom_.header.stamp = current_time_;
// 	odom_.header.frame_id = "base_link";
// 	odom_.pose.pose.position.x = Pickable::x_position_;
// 	odom_.pose.pose.position.y = Pickable::y_position_;
// 	odom_.pose.pose.position.z = Pickable::z_position_;//10/(shapes[i]->getArea()*0.055555);
// 	odom_.pose.pose.orientation = geometry_msgs_quat_;
// 	odom_.child_frame_id = Pickable::tf_link_;//oss.str();
// 	odom_.twist.twist.linear.x = 0.0;//vx;
// 	odom_.twist.twist.linear.y = 0.0;//vy;
// 	odom_.twist.twist.angular.z = 0.0;//vth;
// 	publisher.publish(odom_);
	
	last_time_ = current_time_;
}