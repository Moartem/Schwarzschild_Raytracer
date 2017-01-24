/*
* RasterFunction.h
*
*  Created on: 01.03.2016
*      Author: Pingu
*/

#ifndef RASTERFUNCTION_H_
#define RASTERFUNCTION_H_
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

class RasterFunction180
{
	unsigned int nrNodes;
	double* values;

	RasterFunction180(const RasterFunction180 & a) { RasterFunction180(2); }
public:
	//enum { NO_VALUE = 0xDEADBEEF};
	static double NO_VALUE;

	//Creates an empty Raster Function with a specified number of nodes.
	//Precondition: inNrNodes must be greater than 2
	RasterFunction180(unsigned int inNrNodes) : nrNodes(inNrNodes)
	{
		if (nrNodes < 2)
			throw;
		values = new double[nrNodes];
		resetValues();
	}

	~RasterFunction180()
	{
		delete [] values;
	}

	//resets the number of nodes and deletes all current information
	//Precondition: inNrNodes must be greater than 2
	void resetNrNodes(unsigned int nr)
	{
		if (nrNodes < 2)
			throw;
		nrNodes = nr;
		delete [] values;
		values = new double[nr];
		resetValues();
	}

	//sets all entries to NO_VALUE
	void resetValues()
	{
		for (unsigned int i = 0; i < nrNodes; i++)
			values[i] = NO_VALUE;
	}

	//returns the nr of Nodes
	unsigned int getNrNodes()
	{
		return nrNodes;
	}

	//returns the position of the node nr in the interval [0, pi]
	double PositionOfNode(unsigned int nr)
	{
		return M_PI_2 - nr * M_PI / (nrNodes - 1);
	}

	//writes a value to a specified node
	void writeValue(double value, unsigned int nr)
	{
		if (nr >= nrNodes)
			throw;
		values[nr] = value;
	}

	//returns the value in the node with the number nr
	//Attention! numeration starts with 0 like in arrays
	double valueAt(unsigned int nr)
	{
		if(nr < nrNodes)
			return values[nr];
		return NO_VALUE;
	}

	//gives the interpolated value of f(x) for x in the range of [0, pi]
	double interpolatedValue(double x)
	{
		//x must lie within range
		if (x < -M_PI_2 || x > M_PI_2)
			return NO_VALUE;

		//linear interpolation between the neightboring nodes
		double normedX = (M_PI_2 - x) / M_PI * (nrNodes - 1);
		double leftNode = values[(unsigned int)floor(normedX)];
		double rightNode = values[(unsigned int)ceil(normedX)];
		double weight = normedX - floor(normedX);
		return (leftNode == NO_VALUE || rightNode == NO_VALUE) ?
			NO_VALUE : leftNode * (1 - weight) + rightNode * weight;
	}

	//prints all important information
	void Print(ostream& os)
	{
		for (unsigned int i = 0; i < nrNodes; i++)
			os << '{' << PositionOfNode(i) << ',' << valueAt(i) << '}' << ',' << endl;
		os << endl;
	}

	void toFloatArray(float* in)
	{
		for (unsigned int i = 0; i < nrNodes; i++)
			in[2 * i + 1] = in[2 * i] = values[i];//OGL Hack!!
	}
};

double RasterFunction180::NO_VALUE = -100000.;  //Modify in main!

#endif // !RasterFunction_h_


