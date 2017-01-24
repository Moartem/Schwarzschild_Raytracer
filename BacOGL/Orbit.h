#ifndef ORBIT_H_
#define ORBIT_H_
#pragma once

#include <math.h>
#include "RotMatrix3D.h"
#include "TransUtilities.h"
#include <string>

typedef double Vec3D[3];

class Orbit
{
	double r, schwarzR, orbitAngle, tiltAngle, firstPhi, E, L, u, uBar;
	Vec3D* positionRef;
	Vec3D* orbitMovementRef;
	bool ready;
	RotMatrix3D planeTilt;
	TransUtilities util;
	Vec3D internalCarthPosition, dummy;
	Vec2D polarPos;
public:
	Orbit(Vec3D* position, Vec3D* orbitMovement): positionRef(position), 
		orbitMovementRef(orbitMovement)	{}

	//Not responsible for the references
	~Orbit() {}

	bool bounce;

	bool initNewOrbit(Vec3D & desiredDirection, double inL, double inSchwarzR)
	{
		schwarzR = inSchwarzR;
		r = (*positionRef)[0];
		//Not able to initialize inside
		if (r < schwarzR)
			return false;
		u = 1 / r;
		uBar = 0;
		L = inL;
		E = sqrt((1 - schwarzR / r) * (1 + L * L / (r * r)));

		util.toCarthesic(*positionRef, internalCarthPosition);
		if (internalCarthPosition[2] < 1e-10)
		{
			tiltAngle = 0;
			firstPhi = 0;
			orbitAngle = (*positionRef)[1];
			planeTilt.setToIdentity();
		}
		else
		{
			Vec3D planeNormal, planeNormalProjection, intersection;
			Vec3D z = { 0,0,1 };
			util.crossProduct(internalCarthPosition, desiredDirection, planeNormal);
			if (planeNormal[2] < 1e-10)
			{
				intersection[1] = planeNormal[0];
				intersection[0] = -planeNormal[1];
				intersection[2] = 0;
			}
			else
			{
				planeNormalProjection[0] = planeNormal[0];
				planeNormalProjection[1] = planeNormal[1];
				planeNormalProjection[2] = 0;
				if (planeNormal[2] > 0)
					util.crossProduct(planeNormal, planeNormalProjection, intersection);
				else
					util.crossProduct(planeNormalProjection, planeNormal, intersection);
			}
			orbitAngle = util.angle(intersection, internalCarthPosition);
			if (internalCarthPosition[2] < 0)
				orbitAngle = 2 * M_PI - orbitAngle;
			firstPhi = atan2(intersection[1], intersection[0]);
			tiltAngle = util.angle(planeNormal, z);
			util.yieldPlaneRotMatrix(-tiltAngle, planeTilt);
		}
		ready = true;
		return true;
	}

	
	void doStep(double timeStep)
	{
		if (!ready)
			throw -1;
		
		double aPhi, bPhi, cPhi, aU, bU, cU, aUBar, bUBar, cUBar,
			nextU, nextUBar, deltaPhi;

		//Dirty Runge Kutta
		/*aPhi = timeStep / 2 * L * u * u;
		aU = u + aPhi * uBar;
		aUBar = uBar + aPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * u * u) - u);

		bPhi = timeStep / 2 * L * aU * aU;
		bU = u + bPhi * aUBar;
		bUBar = uBar + bPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * aU * aU) - aU);

		cPhi = timeStep * L * bU * bU;
		cU = u + cPhi * bUBar;
		cUBar = uBar + cPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * bU * bU) - bU);

		deltaPhi = timeStep * L / 2 * (u * u / 6 + aU * aU / 3 + bU * bU / 3 + cU * cU / 6);
		nextU = u + deltaPhi * (uBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
		nextUBar = uBar + deltaPhi * (schwarzR / (2 * L*L) + 
			3 * schwarzR / 2 * (u * u / 6 + aU * aU / 3 + bU * bU / 3 + cU * cU / 6)
			-(u + 2 * aU + 2 * bU + cU) / 6);*/

		//Approximate the angle step according to the time step
		deltaPhi = timeStep * L * u * u / 2;
		nextU = u + deltaPhi * uBar;
		deltaPhi = timeStep * L / 4 * (u * u + nextU * nextU);
		nextU = u + deltaPhi * uBar;
		deltaPhi = timeStep * L / 4 * (u * u + nextU * nextU);
		nextU = u + deltaPhi * uBar;
		deltaPhi = timeStep * L / 4 * (u * u + nextU * nextU);

		unsigned int stepFragments = 1 + (unsigned int)(deltaPhi / 0.001);

		if (stepFragments > 1000000)
			stepFragments = 1;
		if (stepFragments != 1)
			for (unsigned int i = 0; i < stepFragments; i++)
				doStep(timeStep / stepFragments);
		else
		{
			aU = u + deltaPhi / 2 * uBar;
			aUBar = uBar + deltaPhi / 2 * (schwarzR * (1 / (2 * L *L) + 3 / 2 * u * u) - u);
			bU = u + deltaPhi / 2 * aUBar;
			bUBar = uBar + deltaPhi / 2 * (schwarzR * (1 / (2 * L *L) + 3 / 2 * aU * aU) - aU);
			cU = u + deltaPhi * bUBar;
			cUBar = uBar + deltaPhi * (schwarzR * (1 / (2 * L *L) + 3 / 2 * bU * bU) - bU);
			nextU = u + deltaPhi * (uBar / 6 + aUBar / 3 + bUBar / 3 + cUBar / 6);
			nextUBar = uBar + deltaPhi * (schwarzR / (2 * L*L) +
				3 * schwarzR / 2 * (u * u / 6 + aU * aU / 3 + bU * bU / 3 + cU * cU / 6)
				- (u + 2 * aU + 2 * bU + cU) / 6);

			u = nextU;
			uBar = nextUBar;
			r = 1 / u;
			orbitAngle += deltaPhi;
			if (isnan(r))
				r = 0;


			//Calculating and saving total position
			polarPos[1] = 0;
			polarPos[0] = orbitAngle;
			util.polarTransform(polarPos, internalCarthPosition, dummy, planeTilt);
			(*positionRef)[0] = r;
			(*positionRef)[1] = polarPos[0] + firstPhi;
			(*positionRef)[2] = polarPos[1];
		}
	}

	//saves the represented orbiting spectator to spec
	void readOrbitSpectator(Vec3D & spec)
	{
		if (!ready)
			throw -1;
		spec[0] = E / (1 - schwarzR / r);
		spec[1] = (uBar > 0 ? -1 : 1) *
			sqrt(E * E - (1 - schwarzR / r) * (L  *L / (r * r) + 1));
		spec[2] = L / (r * r);
	}

	double tiltCorrectionAngle()
	{
		if (!ready)
			throw -1;
		return tiltAngle * cos((*positionRef)[1] - firstPhi);
	}

	string isStable()
	{
		if (L*L < 3 * schwarzR * schwarzR)
			return "Instable";
		double r1 = L * L / schwarzR * (1 - sqrt(1 - 3 * schwarzR * schwarzR / (L * L)));
		if (r < r1)
			return "Instable";
		if (E > 1)
			return "Escape";
		if (E * E - (1 - schwarzR / r1) * (L * L / (r1 * r1) + 1) < 0)
			return "Stable orbit";
		return "Instable";
	}
};

#endif //!ORBIT_H_