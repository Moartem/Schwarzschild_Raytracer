#ifndef SPHEREGRAPHIC_H_
#define SPHEREGRAPHIC_H_
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "RasterFunction.h"

class SphereGraphic
{

public:
	//currently only produces a checkerboard sphere
	unsigned int colour(Vec2D & pos)
	{
		if (pos[1] == RasterFunction180::NO_VALUE)
			return 0;
		if (fabs(pos[1]) < M_PI / 40)
			return 255 << 8;
		if (fabs(pos[0]) < M_PI / 40)
			return (127 << 8) + (255 << 16);
		bool horizontal = ((int)floor(pos[0] / M_PI * 16)) % 2 != 0;
		bool vertical = ((int)floor(pos[1] / M_PI * 8)) % 2 != 0;
		if (horizontal ^ vertical)
			return 255;
		else
			return 127;
	}

};

#endif //SPHEREGRAPHIC_H_