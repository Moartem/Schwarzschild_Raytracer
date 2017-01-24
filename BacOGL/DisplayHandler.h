#ifndef DISPLAYHANDLER_H_
#define DISPLAYHANDLER_H_
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
typedef double Vec2D[2];

class DisplayHandler
{
	//determines the display
	unsigned int width;
	unsigned int height;
	double fov;

	//derived values
	double widthHalf;
	double heightHalf;
	double scale;
public:
	//Constructor
	DisplayHandler(unsigned int inWidth, unsigned int inHeight, double inFov) :
		width(inWidth), height(inHeight), fov(inFov)
	{
		if (inFov >= M_PI)
			fov = M_PI_2;// cout << "Too large fov" << endl;
		widthHalf = width / 2.;
		heightHalf = height / 2.;
		scale = tan(fov / 2) / widthHalf;
	}

	//sets new parameters for the display
	void setDisplay(unsigned int inWidth, unsigned int inHeight, double inFov = 0)
	{
		width = inWidth;
		height = inHeight;
		if(inFov)
			fov = inFov;
		widthHalf = width / 2.;
		heightHalf = height / 2.;
		scale = tan(fov / 2) / widthHalf;
	}

	//returns the polar coordinates for the pixel (x,y)
	void polarCoordinates(unsigned int x, unsigned int y, Vec2D & outCoords)
	{
		outCoords[0] = atan2(y - heightHalf, x - widthHalf);// - M_PI_2;
		outCoords[1] = M_PI_2 - atan(sqrt((x - widthHalf) * (x - widthHalf) + 
			(y - heightHalf) * (y - heightHalf)) * scale);
	}
};

#endif // !DISPLAYHANDLER_H_

