#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <iostream>
#include <math.h>
#include <typeinfo>
#include <vector>
#include "shape.h"
#include "square.h"
#include "circle.h"
#include "triangle.h"



//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 25*25;//20*20
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
static const std::string OPENCV_WINDOW_ORIGINAL_IMAGE = "Original Image";
static const std::string OPENCV_WINDOW_HSV_IMAGE = "HSV Image";
static const std::string OPENCV_WINDOW_THRESHOLDED_IMAGE = "Thresholded Image";

//Matrix to store each frame of the webcam feed
cv::Mat img_camera;
//matrix storage for HSV image
cv::Mat img_HSV;
//matrix storage for binary threshold image
cv::Mat img_threshold;
//x and y values for the location of the object

const std::string list_colors[] = {"dark blue","soft blue","yellow","red","pink"};
std::string color = "";
//Shape *shape;
std::vector <Shape*> shapes;

ros::Publisher odom_pub;

ros::Time current_time;
ros::Time last_time;

tf::TransformBroadcaster *odom_broadcaster;



int HSV_values[1][6] =
{
    {94, 118, 132, 256, 113, 256},//dark blue 106, 122, 106, 256, 0,  256
    //{96, 104, 104, 166, 34,  256},//soft blue
    //{0, 53, 69, 122, 74,  256},//yellow
    //{0, 96, 126, 256, 0,  256},//red
    //{135, 256, 118, 164, 47,  256},//pink
};


//Function where the binary image is processed to reduce the noice on it
void reduce_noice_from_image(cv::Mat &img_thresholded){
    //Here the image is cleaned from the noice, this could be acomplished in several ways, with applying 
    //cv::erode few times and after cv::dilate also few times, or using cv::blur or cv::GaussianBlur few times.
	
	
	//After running several examples with code in the file gray_canny_with_gaussianblur.cpp that is in the folder example4, I found that the maximal noice reduction is given
	//after applying twice cv::pyrDown and then applying twice cv::GaussianBlur with the parameters used. 
	//For the color recognition using the webcam from the laptop, the application runs well without having to apply the cv::pyrDown twice or onces.
	//BE AWARE THAT FOR USING THE CAMERA OF THE ROBOT IT MAY BE NECESARY TO APPLY THE cv::pyrDown AT LEAST ONCE TO REDUCE THE NOICE.
	cv::GaussianBlur( img_thresholded, img_thresholded, cv::Size(13,13), 1, 1);  
    cv::GaussianBlur( img_thresholded, img_thresholded, cv::Size(3,3), 1, 1);

}

void delete_shapes(){
	for(size_t i=0; i<shapes.size(); i++){
		delete shapes[i];
	}
	shapes.clear();
}

void draw_shapes(cv::Mat &img_camera){
	for(size_t i=0; i<shapes.size(); i++){
		shapes[i]->draw(img_camera);
	}
}


void track_filtered_object(cv::Mat img_threshold, cv::Mat &img_camera){
	
	double hu[7];//the hu moments will be stored in here
	std::string object_type = "";
	int area = 0;
	int m_inertia = 0;
	int x=0, y=0;
	double orientation = 0;
	
	
	cv::Mat temp;
	img_threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	cv::vector< cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	cv::findContours(temp,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE );//cv::CHAIN_APPROX_SIMPLE
	//use moments method to find our filtered object
	
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				cv::Moments moment = moments((cv::Mat)contours[index]);
				area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA){
                    cv::HuMoments(moment,hu);
                    orientation = (180/M_PI)*(atan2((2*moment.mu11),(moment.mu20-moment.mu02)));                    
                    m_inertia = (int)(trunc(round(hu[0]*1000)));
					x = moment.m10/area;
					y = moment.m01/area;
					//std::cout << "The orientation1 is: " << orientation <<std::endl;
					// std::cout << "The position of the first point in x1 is: " << contours[index][0].x << " and in y1 is: " << contours[index][0].y << std::endl;
					int pt_1_4 = contours[index].size()/4;
					// std::cout << "The position of the first point in x2 is: " << contours[index][pt].x << " and in y2 is: " << contours[index][pt].y << std::endl;
					int pt_1_2 = contours[index].size()/2;
					// std::cout << "The position of the first point in x3 is: " << contours[index][pt].x << " and in y3 is: " << contours[index][pt].y << std::endl;
					int pt_3_4 = (3 * contours[index].size())/4;
					// std::cout << "The position of the first point in x4 is: " << contours[index][pt].x << " and in y4 is: " << contours[index][pt].y << std::endl;
					// std::cout << "The x is: " << x << " The y is: " << y << " the area is: " << area <<std::endl;
					cv::Point p1 = contours[index][0];
					cv::Point p2 = contours[index][pt_1_4];
					cv::Point p3 = contours[index][pt_1_2];
					cv::Point p4 = contours[index][pt_3_4];
					
					double orientation1 = (180/M_PI)*(atan2(y-p1.y,x-p1.x));
					
					cv::circle(img_camera,p1,6,cv::Scalar(0,0,255),-1);
					
				
					if(m_inertia <= 166 && m_inertia >= 164){//165
					    shapes.push_back(new Square(x,y,orientation1,m_inertia,area,"Square",color));
					    
					    
    					//int x2 = x - l;
    					//int y2 = y - l;
					    //cv::line(img_camera,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(0,0,255),7);
					}
					else if(m_inertia <= 160 && m_inertia >= 158){//159
					    shapes.push_back(new Circle(x,y,1.0,m_inertia,area,"Circle",color));
					}
					else if((trunc(round(m_inertia/10))) == 18){
					    shapes.push_back(new Triangle(x,y,orientation1,m_inertia,area,"Triangle",color));
					}
					else{
					    shapes.push_back(new Shape(x,y,1.0,m_inertia,area,"Not identified",color));
					}
					//cv::putText(img_camera,shape->shapeInfo(),cv::Point(0,50),2,1,cv::Scalar(0,255,0),2);
					//shape->draw(img_camera);
					
				}

			}

		}else{
			cv::putText(img_camera,"TOO MUCH NOISE!, FILTER IMAGE",cv::Point(0,50),1,2,cv::Scalar(0,0,255),2);
			delete_shapes();
			
		}
	}
	
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
     try{  
		
        std_msgs::Header h;
        h.stamp = ros::Time::now();
        img_camera = cv_bridge::toCvShare(msg, "bgr8")->image;
       
        
        
        /* here the code of the guy START */
        
        //convert frame from BGR to HSV colorspace
        
		reduce_noice_from_image(img_camera);
		cv::cvtColor(img_camera,img_HSV,cv::COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		
		delete_shapes();
		for(int i=0; i<1; i++){
			color = list_colors[i];
			cv::inRange(img_HSV,cv::Scalar(HSV_values[i][0],HSV_values[i][2],HSV_values[i][4]),cv::Scalar(HSV_values[i][1],HSV_values[i][3],HSV_values[i][5]),img_threshold);
			track_filtered_object(img_threshold,img_camera);
		}
		draw_shapes(img_camera);
		
		
		
		
		//Here the vector is checked if it is not empty, if not, then the broadcaster will be used.
		if(!shapes.empty()){//must be replaced by a while statement to let the rest of the members of the vector be shown also.
			
			for(size_t i=0; i<shapes.size(); i++){
			
			
				//Here the variables for the use of TF are declared 
				//ros::Publisher odom_pub;
				//tf::TransformBroadcaster odom_broadcaster;
	
				current_time = ros::Time::now();
				
				//since all odometry is 6DOF we'll need a quaternion created from yaw
			    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw((shapes[i]->getOrientation()*M_PI)/180);
			    //tf::Quaternion odom_quat;
			    //odom_quat.setRPY(0, 0, shapes[0]->getOrientation());
			
			    //first, we'll publish the transform over tf, it will publish the Tf as a TransformStamped with all the needed data about time, position, orientation, frame and child frame
			    geometry_msgs::TransformStamped odom_trans;
			    //tf::TransformStamped odom_trans;
			    odom_trans.header.stamp = current_time;
			    std::ostringstream oss;
				oss << "base_link" << i;
			    odom_trans.header.frame_id = "odom";
			    odom_trans.child_frame_id = oss.str();
			
			    odom_trans.transform.translation.x = shapes[i]->getXPosition()*0.055555;
			    odom_trans.transform.translation.y = shapes[i]->getYPosition()*0.055555;
			    odom_trans.transform.translation.z = 10/(shapes[i]->getArea()*0.055555);
			    odom_trans.transform.rotation = odom_quat;
			    //odom_trans.setRotation(odom_quat);
			    
			    //std::cout << "The values are "<< shapes[i]->getXPosition()*0.055555 << "  " << shapes[i]->getYPosition()*0.055555 << "  " << shapes[i]->getOrientation() <<std::endl;
			
			    //send the transform
			    odom_broadcaster->sendTransform(odom_trans);//(tf::StampedTransform(odom_trans, ros::Time::now(), "odom", "base_link"));
			
			    //next, we'll publish the odometry message over ROS
			    nav_msgs::Odometry odom;
			    odom.header.stamp = current_time;
			    odom.header.frame_id = "odom";
			
			    //set the position
			    odom.pose.pose.position.x = shapes[i]->getXPosition()*0.055555;
			    odom.pose.pose.position.y = shapes[i]->getYPosition()*0.055555;
			    odom.pose.pose.position.z = 10/(shapes[i]->getArea()*0.055555);
			    odom.pose.pose.orientation = odom_quat;
			
			    //set the velocity
			    odom.child_frame_id = oss.str();
			    odom.twist.twist.linear.x = 0.0;//vx;
			    odom.twist.twist.linear.y = 0.0;//vy;
			    odom.twist.twist.angular.z = 0.0;//vth;
			
			    //publish the message
			    odom_pub.publish(odom);
			    
			    
			    last_time = current_time;
			}
    
		}
		
		
		
		
		
		
		
		//show frames 
		cv_bridge::CvImage cv_ptr2(h, sensor_msgs::image_encodings::BGR8, img_threshold);
		cv::imshow(OPENCV_WINDOW_THRESHOLDED_IMAGE,cv_ptr2.image);
		cv_bridge::CvImage cv_ptr(h, sensor_msgs::image_encodings::BGR8, img_camera);
		cv::imshow(OPENCV_WINDOW_ORIGINAL_IMAGE,cv_ptr.image);
		cv_bridge::CvImage cv_ptr1(h, sensor_msgs::image_encodings::BGR8, img_HSV);
		cv::imshow(OPENCV_WINDOW_HSV_IMAGE,cv_ptr1.image);
		
		//delete shape;
		//shape = nullptr;
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition_and_tracking_with_shapes");//color_object_recognition_and_tracking
    ros::NodeHandle nh;
    cv::namedWindow( OPENCV_WINDOW_ORIGINAL_IMAGE, cv::WINDOW_AUTOSIZE );
    cv::namedWindow( OPENCV_WINDOW_HSV_IMAGE, cv::WINDOW_AUTOSIZE );
    cv::namedWindow( OPENCV_WINDOW_THRESHOLDED_IMAGE, cv::WINDOW_AUTOSIZE );
    cv::startWindowThread();
    
    odom_broadcaster = new tf::TransformBroadcaster();
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    //ros::Publisher  image_pub_ = nh.advertise<sensor_msgs::Image>("/image_converter/output_video", 1);
    image_transport::ImageTransport it(nh);//   "/usb_cam/image_raw" "/cameras/left_hand_camera/image"
    image_transport::Subscriber sub = it.subscribe(/*"/image_rect_color"*/"/usb_cam/image_raw", 1, imageCallback);
    
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    ros::spin();
    std::cout << "The spinning is over!" <<std::endl;
    delete_shapes();
    cv::destroyWindow(OPENCV_WINDOW_ORIGINAL_IMAGE);
    cv::destroyWindow(OPENCV_WINDOW_HSV_IMAGE);
    cv::destroyWindow(OPENCV_WINDOW_THRESHOLDED_IMAGE);
}