#ifndef OUTPUTHANDLER_H_
#define OUTPUTHANDLER_H_
#pragma once

//#include <Windows.h>
//#include <gdiplus.h>
#include "CImg.h"

class OutputHandler
{

public:
	void draw(unsigned int colour, unsigned int x, unsigned int y)
	{
		
		//COLORREF myColour = colour;
		//SetPixelV(screen, x, y, myColour);
		char out;
		switch (colour)
		{
		case 0:
			out = ' ';
			break;
		case 255:
			out = 'x';
			break;
		case 127:
			out = '*';
			break;
		case 1:
			out = '+';
			break;
		default:
			out = ' ';

		}
		cout << out;
		if (x == 149)
			cout << endl;
	}
};

#endif //OUTPUTHANDLER_H_