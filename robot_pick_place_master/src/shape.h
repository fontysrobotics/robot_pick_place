/*Here the shape class must be defined and they should be between certain high, each shape will inheritate from the shape class, and each shape will have

at least, a center of mass position, an orientation, Hu moment of inertia and color, check wich design pattern best fit in this situation.
A class Color has to be created to specify the Shape's color*/
#ifndef SHAPE_H
#define SHAPE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <math.h>
#include <iostream>
#include <sstream>

class Shape{
    public:
    	Shape(int xPos, int yPos, int orien, int mOfInertia, int a, std::string t, std::string c);
    	Shape();
    	virtual ~Shape();//virtual ~Shape(void); //look for an example of a virtual destructor
    	int getArea();
    	void setArea(int a);
    	int getXPosition();
    	void setXPosition(int xPos);
    	int getYPosition();
    	void setYPosition(int yPos);
    	int getOrientation();
    	void setOrientation(int orien);
    	int getMomentOfInertia();
    	void setMomentOfInertia(int mOfInertia);
    	std::string getType();
    	void setType(std::string t);
    	std::string getColor();
    	void setColor(std::string c);
    	virtual void draw(cv::Mat &);//virtual void draw();
    	std::string shapeInfo();
    	void setViewShape(int xPos, int yPos, int orien, int mOfInertia, int a);
    	void createModel(std::string frame_id);
    private:
    	int area;
    	int xPosition;
    	int yPosition;
    	int orientation;
    	int momentOfInertia;
    	std::string type;
    	std::string color;
};

#endif
