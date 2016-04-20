#include "shape.h"


Shape::Shape(int xPos, int yPos, int orien, int mOfInertia, int a, std::string t, std::string c){
	Shape::setArea(a);
	Shape::setXPosition(xPos);
	Shape::setYPosition(yPos);
	Shape::setOrientation(orien);
	Shape::setMomentOfInertia(mOfInertia);
	Shape::setType(t);
	Shape::setColor(c);
}

Shape::Shape(){
	Shape::setArea(0);
	Shape::setXPosition(-1);
	Shape::setYPosition(-1);
	Shape::setOrientation(0);
	Shape::setMomentOfInertia(0);
	Shape::setType("");
	Shape::setColor("");	
}

Shape::~Shape(){
   //std::cout << "Destructor of Shape called!" <<std::endl; 	
}

int Shape::getArea(){
	return Shape::area;
}

void Shape::setArea(int a){
	if(a>0.0)
		Shape::area = a;
	else
		Shape::area = 0.0;
}

int Shape::getXPosition(){

	return Shape::xPosition;

}

void Shape::setXPosition(int xPos){

	Shape::xPosition = xPos;

}

int Shape::getYPosition(){

	return Shape::yPosition;

}

void Shape::setYPosition(int yPos){

	Shape::yPosition = yPos;

}

int Shape::getOrientation(){
	return orientation;
}

void Shape::setOrientation(int orien){
	Shape::orientation = orien;
}

int Shape::getMomentOfInertia(){
	return momentOfInertia;
}

void Shape::setMomentOfInertia(int mOfInertia){
	Shape::momentOfInertia = mOfInertia;
}

std::string Shape::getType(){
	return Shape::type;
}

void Shape::setType(const std::string t){
	Shape::type = t;
}

std::string Shape::getColor(){
	return Shape::color;
}

void Shape::setColor(const std::string c){
	Shape::color = c;
}

void Shape::draw(cv::Mat &frame){
	int FRAME_WIDTH = 640;
	int FRAME_HEIGHT = 480;
	int x = Shape::getXPosition();
    int y = Shape::getYPosition();
    std::stringstream ssx;
    std::stringstream ssy; 
    
    if(x>=0 && y>=0){
		cv::circle(frame,cv::Point(x,y),20,cv::Scalar(0,255,0),2);
	    if(y-25>0)
	        cv::line(frame,cv::Point(x,y),cv::Point(x,y-25),cv::Scalar(0,255,0),2);
	    else 
	        cv::line(frame,cv::Point(x,y),cv::Point(x,0),cv::Scalar(0,255,0),2);
	    if(y+25<FRAME_HEIGHT)
	        cv::line(frame,cv::Point(x,y),cv::Point(x,y+25),cv::Scalar(0,255,0),2);
	    else 
	        cv::line(frame,cv::Point(x,y),cv::Point(x,FRAME_HEIGHT),cv::Scalar(0,255,0),2);
	    if(x-25>0)
	        cv::line(frame,cv::Point(x,y),cv::Point(x-25,y),cv::Scalar(0,255,0),2);
	    else 
	        cv::line(frame,cv::Point(x,y),cv::Point(0,y),cv::Scalar(0,255,0),2);
	    if(x+25<FRAME_WIDTH)
	        cv::line(frame,cv::Point(x,y),cv::Point(x+25,y),cv::Scalar(0,255,0),2);
	    else 
	        cv::line(frame,cv::Point(x,y),cv::Point(FRAME_WIDTH,y),cv::Scalar(0,255,0),2);
	
		ssx << x;
		ssy << y;	
		cv::putText(frame,ssx.str()+","+ssy.str(),cv::Point(x,y+30),1,1,cv::Scalar(0,255,0),2);
    }
}

std::string Shape::shapeInfo(){
	return Shape::getType() + " " + Shape::getColor();
}

void Shape::setViewShape(int xPos, int yPos, int orien, int mOfInertia, int a){
	Shape::setArea(a);
	Shape::setXPosition(xPos);
	Shape::setYPosition(yPos);
	Shape::setOrientation(orien);
	Shape::setMomentOfInertia(mOfInertia);
}
//Has the baseLink of the URDF file with the shape.
void Shape::createModel(std::string frame_id){
	//here the urdf file must be open, then the first line must be read and the value of the 
	//baselink name must be changed by the one i frame_id 
}