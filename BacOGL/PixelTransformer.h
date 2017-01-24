#ifndef PIXELTRANSFORMER_H_
#define PIXELTRANSFORMER_H_
#pragma once

#include "RotMatrix3D.h"
#include "RasterFunction.h"

class PixelTransformer
{
	double psiSqrt, psiMinus1Sqrt;

	//Transforms a sphere vector with radius 1 to carthesic coordinates
	void toCarthesic(Vec2D & pVec, Vec3D & cartVec)
	{
		cartVec[0] = cos(pVec[0])*cos(pVec[1]);
		cartVec[1] = sin(pVec[0])*cos(pVec[1]);
		cartVec[2] = sin(pVec[1]);
	}

	//Transforms a carthesic Vector to sphere coordinates
	//r is assumed to be 1.
	void toPolar(Vec3D & cartVec, Vec2D & pVec)
	{
		pVec[0] = atan2(cartVec[1], cartVec[0]);
		pVec[1] = asin(cartVec[2]);
	}

	//Transforms the given polar coordinates to different polar coordinates
	//specified by rotator
	//This function and all dependencies are GPU critical
	void polarTransform(Vec2D & original, Vec3D & dummy1, Vec3D & dummy2, RotMatrix3D & rotator)
	{
		toCarthesic(original, dummy1);
		rotator.transform(dummy1, dummy2);
		toPolar(dummy2, original);
	}

public:
	//Nothing to do
	PixelTransformer() {}

	RotMatrix3D firstTransform;
	RotMatrix3D secondTransform;
	RotMatrix3D thirdTransform;
	RasterFunction180* geodesicRef;

	void setPsi(double psi)
	{
		psiSqrt = sqrt(psi);
		psiMinus1Sqrt = sqrt(psi - 1);
	}

	void transformPixel(Vec2D & toTrans, Vec3D& dummy1, Vec3D& dummy2)
	{
		polarTransform(toTrans, dummy1, dummy2, firstTransform);
		if (psiSqrt > 1.000001)
		{
			double sinResult = sin(toTrans[1]);
			toTrans[1] = asin((-psiMinus1Sqrt + sinResult * psiSqrt) /
				(psiSqrt - sinResult * psiMinus1Sqrt));
		}
		polarTransform(toTrans, dummy1, dummy2, secondTransform);
		toTrans[1] = geodesicRef->interpolatedValue(toTrans[1]);

		//Check if it hit the surface
		if (toTrans[1] != geodesicRef->NO_VALUE)
			polarTransform(toTrans, dummy1, dummy2, thirdTransform);
	}
};

#endif //PIXELTRANSFORMER_H_
