#ifndef TRANSUTILITIES_H_
#define TRANSUTILITIES_H_
#pragma once

#define _USE_MATH_DEFINES
#include "RotMatrix3D.h"

class TransUtilities
{
public:
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

	void toCarthesic(Vec3D & pVec, Vec3D & cartVec)
	{
		cartVec[0] = pVec[0] * cos(pVec[1])*cos(pVec[2]);
		cartVec[1] = pVec[0] * sin(pVec[1])*cos(pVec[2]);
		cartVec[2] = pVec[0] * sin(pVec[2]);
	}

	void toPolar(Vec3D & cartVec, Vec3D & pVec)
	{
		pVec[0] = sqrt(cartVec[0] * cartVec[0] + cartVec[1] * cartVec[1] 
			+ cartVec[2] * cartVec[2]);
		pVec[1] = atan2(cartVec[1], cartVec[0]);
		pVec[2] = asin(cartVec[2] / pVec[0]);
	}

	//saves the matrix of basis vectors with third vector equal to Axes
	//to the target matrix.
	//Not GPU critical
	void yieldTransMatrix(Vec2D & axis, RotMatrix3D & target)
	{
		Vec3D z;
		toCarthesic(axis, z);

		//turning z 90 degree down yields x
		Vec2D xPolar;
		//toPolar(z, zToXPolar);
		xPolar[1] = axis[1] - M_PI_2;
		xPolar[0] = axis[0];
		Vec3D x;
		toCarthesic(xPolar, x);

		//y is the cross product ZxX
		Vec3D y;
		crossProduct(z, x, y);

		target = RotMatrix3D(x, y, z);
		target.clearArtifacts();
	}

	//Produces the Plane tilt Matrix
	//A Rotation in x, y
	void yieldTiltMatrix(double phi, RotMatrix3D & target)
	{
		Vec3D x, y, z;
		y[1] = x[0] = cos(phi);
		y[0] = sin(phi);
		x[1] = -sin(phi);
		z[2] = 1;
		x[2] = y[2] = z[0] = z[1] = 0;

		target = RotMatrix3D(x, y, z);
		target.transpose();
		target.clearArtifacts();
	}

	//Used in Rotating into movement direction in the yz plane
	void yieldPlaneRotMatrix(double phi, RotMatrix3D & target)
	{
		Vec3D x, y, z;
		y[1] = z[2] = cos(phi);
		z[1] = sin(phi);
		y[2] = -sin(phi);
		x[0] = 1;
		x[2] = y[0] = z[0] = x[1] = 0;
		target = RotMatrix3D(x, y, z);
		target.clearArtifacts();
	}

	//saves axb to result
	void crossProduct(const Vec3D & a, const Vec3D & b, Vec3D & result)
	{
		result[0] = a[1] * b[2] - a[2] * b[1];
		result[1] = a[2] * b[0] - a[0] * b[2];
		result[2] = a[0] * b[1] - a[1] * b[0];
	}

	double angle(const Vec3D & a, const Vec3D & b)
	{
		double scalar = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
		if (fabs(scalar) > 1e-10)
			scalar /= sqrt((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) *
				(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]));
		return acos(scalar);
	}
};

#endif //!TRANSUTILITIES_H_