#ifndef GEODESICSOLVER_H_
#define GEODESICSOLVER_H_
#pragma once

#include "RasterFunction.h"
#include <math.h>
#include <omp.h>

class GeodesicSolver
{
	const double SINGULARITY_BOUND = 1e-10;
	unsigned int maxIterations;
	double standardStep;

	//filters some cases and prepares for solving
	double PrehandlerRKTheta(double E, double L, double rStart,
			bool rFalling, double schwarzR, double surfaceR)
	{
		double b = E ? L / E : 1e20;
		bool outside = rStart > schwarzR;
		bool surfaceOutside = surfaceR > schwarzR;
		bool innerView = rStart < surfaceR;

		//If b = 0 no equations to solve
		if (L < SINGULARITY_BOUND)
		{
			if (!innerView)
				if (surfaceOutside)
					return rFalling ? 0 : RasterFunction180::NO_VALUE;
				else
					return RasterFunction180::NO_VALUE;
			else
				if (outside)
					return rFalling ? (schwarzR ? RasterFunction180::NO_VALUE : M_PI) : 0;
				else
					return surfaceOutside ?
					(E > 0 ? 0 : RasterFunction180::NO_VALUE): 0;
		}
		bool barrier3R_2 = schwarzR &&
			1 / (b * b) < 4 / (27 * schwarzR * schwarzR);
		bool differentSides3R_2 = ((rStart < 3 * schwarzR / 2) ^ (surfaceR < 3 * schwarzR / 2))
			&& fabs(rStart - 3 * schwarzR / 2) > SINGULARITY_BOUND;
		
		//Some cases where the geodesis won´t hit the surface
		if ((!innerView && !surfaceOutside) ||
			(!outside && surfaceOutside && E < 0) ||
			(barrier3R_2 && differentSides3R_2) ||
			(rStart < 3 * schwarzR / 2 && innerView && rFalling) ||
			(rStart > 3 * schwarzR / 2 && !innerView && !rFalling))
		{
			return RasterFunction180::NO_VALUE;
		}

		double u = 1 / rStart;
		double uBar = (rFalling ? 1 : -1) *
			sqrt(1 / (b  * b) - (1 - schwarzR / rStart) / rStart / rStart);
		if (fabs(uBar) < SINGULARITY_BOUND)
			return RasterFunction180::NO_VALUE;
		return RungeKuttaSolver(u, uBar, schwarzR, surfaceR);
	}

	double RungeKuttaSolver(double u, double uBar, double R, double surfaceR)
	{

		double bound = 0.9 * fmin(u, 1 / fmax(surfaceR, 2 * R));
		double step = standardStep;
		unsigned int finishRefinement = 3;
		//if (fabs(uBar) > 1)
		//	step /= fabs(uBar);
		double stepHalf = step / 2;
		double R3_2 = 3 * R / 2;
		double angle = 0;

		double currentU = u;
		double currentUBar = uBar;
		double nextU, nextUBar, aU, aUBar, bU, bUBar, cU, cUBar;
		double surfaceU = 1 / surfaceR;
		unsigned int iteration = 0;
		double solution = 0;

		while ((!(R && currentU > 1 / R && currentUBar > 0)) && //check if the ray is inside B and r is falling
			iteration < maxIterations && currentU > 0)
		{
			aU = currentU + stepHalf * currentUBar;
			aUBar = currentUBar + stepHalf * (-currentU + R3_2 * currentU*currentU);
			bU = currentU + stepHalf * aUBar;
			bUBar = currentUBar + stepHalf * (-aU + R3_2 * aU*aU);
			cU = currentU + step * bUBar;
			cUBar = currentUBar + step * (-bU + R3_2 * bU*bU);

			nextU = currentU + step * (currentUBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
			nextUBar = currentUBar + step * ((-currentU + R3_2 * currentU*currentU) / 6 +
				(-aU + R3_2 * aU*aU) / 3 + (-bU + R3_2 * bU*bU) / 3 + (-cU + R3_2 * cU*cU) / 6);

			//check if the tray has passed through the surface do some newton
			if ((nextU > surfaceU) ^ (u > surfaceU))
			{
				double value, derivitive, newtonStep;
				if (fabs(currentUBar) > fabs(nextUBar))
				{
					newtonStep = 0;
					value = currentU;
					derivitive = currentUBar;
				}
				else
				{
					newtonStep = step;
					value = nextU;
					derivitive = nextUBar;
				}

				for (int i = 0; i < finishRefinement; i++)
				{
					newtonStep -= (value - surfaceU) / derivitive;

					aU = currentU + newtonStep * currentUBar / 2;
					aUBar = currentUBar + newtonStep * (-currentU + R3_2 * currentU*currentU) / 2;
					bU = currentU + newtonStep * aUBar / 2;
					bUBar = currentUBar + newtonStep * (-aU + R3_2 * aU*aU) / 2;
					cU = currentU + newtonStep * bUBar;
					cUBar = currentUBar + newtonStep * (-bU + R3_2 * bU*bU);

					value = currentU + newtonStep * (currentUBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
					derivitive = currentUBar + newtonStep * ((-currentU + R3_2 * currentU*currentU) / 6 +
						(-aU + R3_2 * aU*aU) / 3 + (-bU + R3_2 * bU*bU) / 3 + (-cU + R3_2 * cU*cU) / 6);
				}
				return angle + newtonStep; //+ step * ((surfaceU - u)/(nextU-u)); (for linear interpolation)
			}

			//check if the ray leaves the relevant domain
			if (nextU < bound)
				return RasterFunction180::NO_VALUE;
			currentU = nextU;
			currentUBar = nextUBar;
			iteration++;
			angle += step;
		}
		return RasterFunction180::NO_VALUE;
	}


public:
	//Nothing to do
	GeodesicSolver(double step = M_PI / 100) :
		maxIterations((int)(60/step)), standardStep(step)
	{}

	void solveGeodesic(double schwarzR, double r, 
		double surfaceR, RasterFunction180 & raster)
	{
		unsigned int nodes = raster.getNrNodes();
		
		//modified to run parallel
//#pragma omp parallel for
		for(/*unsigned*/ int i = 0; i < nodes; i++)
		{
			double theta, L, E;
			bool rFalling;
			theta = raster.PositionOfNode(i);
			L = r* cos(theta);
			//for the future path of the light, note that the future path has -theta
			if (r < schwarzR)
			{
				rFalling = false;
				E = sin(-theta) * sqrt(schwarzR / r - 1);
			}
			else
			{
				//checks if they are comming from lower r
				rFalling = sin(theta) > 0;
				E = sqrt(1 - schwarzR / r);
			}
			double result = PrehandlerRKTheta(E, L, r, rFalling, schwarzR, surfaceR);
			//NO_VALUE must be preserved
			raster.writeValue(result == RasterFunction180::NO_VALUE ?
				RasterFunction180::NO_VALUE : M_PI_2 - result, i);
		}
	}
};

#endif //!GEODESICSOLVER_H_