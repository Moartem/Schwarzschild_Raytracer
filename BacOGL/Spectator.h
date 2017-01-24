#ifndef SPECTATOR_H_
#define SPECTATOR_H_

#include <math.h>
#include "RotMatrix3D.h"
#include "RasterFunction.h"
#include "GeodesicSolver.h"
#include "TransUtilities.h"
#include "Orbit.h"
#include "Benchmark.h"

class Spectator
{
	enum SpectatorMode { SCHWARZSCHILD, FREE_FALLING, FIXED_FALLING, FIXED_ORBIT };
	const double MOVEMENT_STEP = .1001;
	const double VERTICAL_RESTRICTION = M_PI_2 - 0.01;
	const double SINGULARITY_BOUND = 1e-10;

	//Information
	Vec3D position, movement; //(r, phi, theta), (t, r, phi)
	Vec2D camera; //sphere-polar
	double E, schwarzR, surfaceR, tau, timeStep;
	bool surfaceBounce, rFalling;
	SpectatorMode mode;

	//dependend output values
	RasterFunction180 geodesics;
	RotMatrix3D displayToMovement, movementToAxis, axisToSurface;
	double psi;

	//depending values
	Vec3D carthesicPosition;
	bool singularityFlag, r0Reached;

	//variables for the central falling verlet solver
	double lastR, nextR;

	//Parts of displayToMovement
	RotMatrix3D yXRotator, displayToStandard,
		standardToAxis, orbitPlaneTilt, tiltedAxisToMovement1;

	//Parts of movementToAxis
	RotMatrix3D movementToTiltedAxis2, orbitPlaneUntilt;

	//Correction on Sphere for geodesic transformation
	RotMatrix3D yInverse;

	//Utilities
	GeodesicSolver solver;
	TransUtilities matUtil;
	Orbit orbit;
	//Benchmark* geodesicBench,* matricesBench;


	//Checks if the system is singular and saves to the flags
	void singularityChecks()
	{
		r0Reached = schwarzR && position[0] / schwarzR < SINGULARITY_BOUND;
		singularityFlag = schwarzR && (r0Reached ||
			fabs(position[0] / schwarzR - 1) < SINGULARITY_BOUND);
		if (position[0] <= schwarzR)
			rFalling = true;
	}

	//calculates h(r), view script for info
	double h()
	{
		return 1 - schwarzR / position[0];
	}

	//saves a Schwarzschild spectator
	void schwarzschildSpectator(Vec3D & spectator)
	{
		if (position[0] > schwarzR)
		{
			spectator[0] = 1 / sqrt(h());
			spectator[1] = 0;
		}
		else
		{
			spectator[0] = 0;
			spectator[1] = -1 * sqrt(-h());
		}
		spectator[2] = 0;
	}

	//saves a central falling spectator
	//if the distance is to high for the given Energy, it saves a Schwarzschild spectator
	void centralFallingSpectator(Vec3D & spectator)
	{
		if (position[0] == schwarzR)
			return;
		if (E * E < h())
			return schwarzschildSpectator(spectator);
		spectator[0] = E / h();
		spectator[1] = -sqrt(E * E - h()) * (rFalling ? 1 : -1);
		spectator[2] = 0;
	}

	//input x, y, up with respect to the camera
	//x, y stay in the horizontal plane
	void doStep(Vec3D & desiredDirection)
	{
		switch (mode)
		{
		case FREE_FALLING:		//Moving into the desired direction
		case SCHWARZSCHILD:
			matUtil.toCarthesic(position, carthesicPosition);
			carthesicPosition[2] += desiredDirection[2] * MOVEMENT_STEP;
			carthesicPosition[0] += (cos(camera[0]) * desiredDirection[0]
				- sin(camera[0]) * desiredDirection[1])	* MOVEMENT_STEP;
			carthesicPosition[1] += (sin(camera[0]) * desiredDirection[0]
				+ cos(camera[0]) * desiredDirection[1])	* MOVEMENT_STEP;
			matUtil.toPolar(carthesicPosition, position);
			break;
		case FIXED_FALLING:		//Following the central geodesic for one time step
			if (!r0Reached)
				doCentralVerletStep();
			break;
		case FIXED_ORBIT:		//Making one time step on the orbit
			if(!r0Reached)
				orbit.doStep(timeStep);
			break;
		default:
			;
		}
		singularityChecks();
	}

	//Performs one time step for a central falling spectator.
	//This is done by Verlet integration.
	void doCentralVerletStep()
	{
		nextR = 2 * position[0] - lastR - timeStep * timeStep * schwarzR /
			(2 * position[0] * position[0]);
		lastR = position[0];
		position[0] = nextR;
		tau += timeStep;
 	}

	//updates the spectator (movement) for the new data
	void updateSpectator()
	{
		switch (mode)
		{
		case SCHWARZSCHILD:
			schwarzschildSpectator(movement);
			break;
		case FIXED_FALLING:
		case FREE_FALLING:
			centralFallingSpectator(movement);
			break;
		case FIXED_ORBIT:
			orbit.readOrbitSpectator(movement);
			break;
		default:
			;
		}
	}
public:
	//TODO , , , modeChangers, getters,

	//Constructor, initializes important components
	Spectator(unsigned int rasterResolution = 2000,	double step = M_PI / 100) :
		geodesics(rasterResolution), position(), movement(),
		orbit(&position, &movement), solver(step)
	{
		position[0] = 10;
		surfaceBounce = false;
		mode = SCHWARZSCHILD;
		tau = 0;
		camera[0] = M_PI;
		camera[1] = 0;

		Vec3D first = { 0,-1,0 };
		Vec3D second = { 1,0,0 };
		Vec3D third = { 0,0,1 };
		yXRotator = RotMatrix3D(first, second, third);
		yInverse = RotMatrix3D(second, first, third);
	}

	//Nothing to do
	~Spectator() {}

	//Sets all necessary parameters, adviced to call before doing anything
	void init(Vec3D inPosition, double inSchwarzR, double inSurfaceR, double phi,
		double theta = 0, double inTimeStep = 1./60, bool inBounce = false)
	{
		setPositionRadius(inPosition, inSchwarzR);
		setSurfaceRadius(inSurfaceR);
		setCamera(phi, theta);
		setPace(inTimeStep);
		setBounce(inBounce);
	}

	//initialices the Benchmarks
	//void initBenchmarks(Benchmark* geoBench, Benchmark* matBench)
	//{
	//	geodesicBench = geoBench;
	//	matricesBench = matBench;
	//}

	//initializes or changes the components that may cause singularities
	void setPositionRadius(Vec3D inPosition, double inSchwarzR)
	{
		position[0] = fabs(inPosition[0]);
		position[1] = inPosition[1];
		position[2] = inPosition[2];
		schwarzR = inSchwarzR;
		singularityChecks();
	}

	//Modifies the radius of the glowing sphere
	void setSurfaceRadius(double inSurfaceR) { surfaceR = inSurfaceR; }

	//Sets where the spectator is looking
	void setCamera(double phi, double theta)
	{
		camera[0] = phi;
		camera[1] = theta;
	}

	//Sets the speed of the simulation
	void setPace(double inTimeStep) { timeStep = inTimeStep; }

	//determines if a fixed falling spectator should bounce off the surface
	void setBounce(bool inBounce) { surfaceBounce = inBounce; }

	bool isSingular() { return singularityFlag; }

	void startOrbit(double inL)
	{
		Vec3D direction = { -sin(position[1]),cos(position[1]),0 };
		orbit.initNewOrbit(direction, inL, schwarzR);
		mode = FIXED_ORBIT;
	}

	//Enters a mode, where the user can move freely and sees,
	//what a spectator would see, who started falling at the specified height
	//Use no argument to start from where you are
	void startFreeFall(double startOfFall = 0)
	{
		if (startOfFall == 0)
			startOfFall = position[0];
		if (startOfFall <= schwarzR * (1 + SINGULARITY_BOUND))
			return;
		mode = FREE_FALLING;
		E = sqrt(1 - schwarzR / startOfFall);
		rFalling = true;
	}

	void startFixedFall()
	{
		if (position[0] <= schwarzR * (1 + SINGULARITY_BOUND))
			return;
		mode = FIXED_FALLING;
		E = sqrt(h());
		lastR = position[0];
		position[0] = lastR - timeStep * timeStep * schwarzR /
			(4 * lastR * lastR);
		tau += timeStep;
		singularityChecks();
	}

	//Switches to schwarzschildMode
	void schwarzschildMode()
	{
		mode = SCHWARZSCHILD;
	}

	//Advances some time steps without rendering images inbetween
	//Only effective in fixed modes
	void advanceInTime(unsigned int numberOfSteps)
	{
		if (mode == FREE_FALLING || mode == SCHWARZSCHILD)
			return;
		Vec3D dummy = { 0,0,0 };
		for (unsigned int i = 0; i < numberOfSteps; i++)
			doStep(dummy);
	}

	//Performs all calculations to advance one step in time 
	//and prepares all transformation parameters/matrices
	void prepare(Vec3D & desiredDirection)
	{
		//updating the position
		doStep(desiredDirection);
		
		//Stop optical calculations if we are singular, only update the camera
		if (singularityFlag)
		{
			matUtil.yieldTransMatrix(camera, displayToStandard);
			displayToMovement.setToIdentity();
			displayToMovement.multiplyOnto(tiltedAxisToMovement1);
			displayToMovement.multiplyOnto(orbitPlaneTilt);
			displayToMovement.multiplyOnto(standardToAxis);
			displayToMovement.multiplyOnto(displayToStandard);
			displayToMovement.multiplyOnto(yXRotator);
			return;
		}
			

		//updates "movement" for further calculations
		updateSpectator();
		
		//Solving the geodesics for the Schwarzschild spectator
		//if(geodesicBench != nullptr)
		//	geodesicBench->startMeasurement();
		solver.solveGeodesic(schwarzR, position[0], surfaceR, geodesics);
		//if (geodesicBench != nullptr)
		//	geodesicBench->stopMeasurement();
		//geodesics.Print(cout);


		//Calculating some matrices
		//if (matricesBench != nullptr)
		//	matricesBench->startMeasurement();
		matUtil.yieldTransMatrix(camera, displayToStandard);

		Vec2D axis = { position[1], position[2] };
		matUtil.yieldTransMatrix(axis, axisToSurface);

		Vec2D invAxis = { axis[0] + M_PI, -axis[1] };
		matUtil.yieldTransMatrix(invAxis, standardToAxis);
		standardToAxis.transpose();

		double r = position[0];
		double planeAngle1, planeAngle2;
		
		
		psi = r > schwarzR ? movement[0] * movement[0] * h()
			: -movement[1] * movement[1] / h();
		if ((psi - 1) < SINGULARITY_BOUND)
			psi = 1;
		//cout << r << endl;
		if (mode == FIXED_ORBIT)
		{
			planeAngle1 = acos(-movement[0] * movement[1] * (r > schwarzR ? 1 : -1) /
				sqrt((1 + r*r*movement[2] * movement[2]) * (psi * (psi - 1))));
			if (r > schwarzR)
			{
				
				planeAngle2 = acos(-movement[1] / sqrt((psi -1) * h()));
			}
			else
			{
				planeAngle2 = acos(-movement[0] * sqrt(-h() / (psi - 1)));
			}
			matUtil.yieldTiltMatrix(orbit.tiltCorrectionAngle(), orbitPlaneTilt);
			matUtil.yieldPlaneRotMatrix(planeAngle1, tiltedAxisToMovement1);
			matUtil.yieldPlaneRotMatrix(planeAngle2, movementToTiltedAxis2);
		}
		else
		{
			orbitPlaneTilt.setToIdentity();
			tiltedAxisToMovement1.setToIdentity();
			movementToTiltedAxis2.setToIdentity();
		}
		orbitPlaneUntilt = orbitPlaneTilt;
		orbitPlaneUntilt.transpose();
		movementToTiltedAxis2.transpose();

		//Composing displayToMovement
		displayToMovement.setToIdentity();
		displayToMovement.multiplyOnto(tiltedAxisToMovement1);
		displayToMovement.multiplyOnto(orbitPlaneTilt);
		displayToMovement.multiplyOnto(standardToAxis);
		displayToMovement.multiplyOnto(displayToStandard);
		displayToMovement.multiplyOnto(yXRotator);

		//Composing movementToAxis
		movementToAxis.setToIdentity();
		movementToAxis.multiplyOnto(orbitPlaneUntilt);
		movementToAxis.multiplyOnto(movementToTiltedAxis2);

		//Correction on Sphere
		axisToSurface.multiplyOnto(yInverse);
		
		//if (matricesBench != nullptr)
		//	matricesBench->stopMeasurement();
	}

	//Turns the camera by specified directions
	void controlCamera(double horizontalAngle, double verticalAngle)
	{
		camera[0] += horizontalAngle;
		camera[1] += verticalAngle;
		//Make sure not to turn over singular point
		camera[1] = fmin(fmax(camera[1], -VERTICAL_RESTRICTION), VERTICAL_RESTRICTION);
		//Prevent too big values
		if (fabs(camera[0]) > 15)
			camera[0] = asin(sin(camera[0]));
	}

	//Writes results to the arguments
	void outputResults(float* firstTransform, float* secondTransform,
		float* thirdTransform, double & psiRef, float* geodesicsTrans)
	{
		displayToMovement.toFloatArray(firstTransform);
		movementToAxis.toFloatArray(secondTransform);
		axisToSurface.toFloatArray(thirdTransform);
		psiRef = psi;
		geodesics.toFloatArray(geodesicsTrans);
	}
};

#endif // !SPECTATOR_H_


/*
	static void normSpectator(double R, double r, Vec2D spectator)
	{
		double normSquare = -spectator[0] * spectator[0] * h(R, r) +
			spectator[1] * spectator[1] / h(R, r);
		if (normSquare >= 0)
			cout << "Error: not a timelike Spectator" << endl;
		spectator[0] /= sqrt(-normSquare);
		spectator[1] /= sqrt(-normSquare);
	}*/

