#pragma once
#ifndef RENDER_H_
#define RENDER_H_

//#include "DisplayHandler.h"
//#include "PixelTransformer.h"
//#include "SphereGraphic.h"
#include "RotMatrix3D.h"
#include "Spectator.h"
#include <omp.h>

//using namespace cimg_library;

class Render
{
	const double CAMERA_STEP = M_PI / 800;

	Spectator spec;
	//Benchmark* geodesicBench, *matricesBench;

	Vec3D controls;
	Vec2D cameraMove;
	
public:
	Render(unsigned int rasterResolution, double lightStep, double R, double inSurfaceR,
		double rStart, double view) : //geodesicBench(nullptr),
		//matricesBench(nullptr), 
		spec(rasterResolution, lightStep)
	{
		Vec3D startPos = { rStart,0,0 };
		spec.init(startPos, R, inSurfaceR, view);
		spec.startFreeFall(rStart * 2);
	}

	~Render()
	{
		//delete geodesicBench, matricesBench;
	}

	//Moves or views according to the command
	void control(char command)
	{
		controls[0] = 0;
		controls[1] = 0;
		controls[2] = 0;
		cameraMove[0] = 0;
		cameraMove[1] = 0;
		switch (command)
		{
		case 'w':
			controls[0] = 1;
			break;
		case 's':
			controls[0] = -1;
			break;
		case 'a':
			controls[1] = 1;
			break;
		case 'd':
			controls[1] = -1;
			break;
		case 'q':
			controls[2] = 1;
			break;
		case 'e':
			controls[2] = -1;
			break;
		case 't':
			cameraMove[1] = CAMERA_STEP;
			break;
		case 'g':
			cameraMove[1] = -CAMERA_STEP;
			break;
		case 'f':
			cameraMove[0] = CAMERA_STEP;
			break;
		case 'h':
			cameraMove[0] = -CAMERA_STEP;
			break;
		case '1':
			spec.schwarzschildMode();
			break;
		case '2':
			spec.startFreeFall(1e10);
			break;
		case '3':
			spec.startFixedFall();
			break;
		case '4':
			double readL;
			cout << "Enter spin:" << endl; //some stable orbit estimates
			cin >> readL;
			spec.startOrbit(readL);
			break;
		case 'z':
			unsigned int nrOfSteps;
			cout << "Enter number of steps:" << endl;
			cin >> nrOfSteps;
			spec.advanceInTime(nrOfSteps);
			break;
		case 'b':
			//printBenchmarks(cout);
			break;
		default:
			;
		
		}
	spec.controlCamera(cameraMove[0], cameraMove[1]);
	}

	void moveCamera(double phi, double theta)
	{
		spec.controlCamera(phi, theta);
	}

	//Enables Benchmarking
	/*void enableBenchmarks()
	{
		geodesicBench = new Benchmark("Geodesic Solving");
		matricesBench = new Benchmark("Calculating transformation matrices");
		spec.initBenchmarks(geodesicBench, matricesBench);
	}

	void printBenchmarks(std::ostream& os)
	{
		geodesicBench->printReport(os);
		matricesBench->printReport(os);
	}*/

	bool isSingular()
	{
		return spec.isSingular();
	}

	//builds one frame and displays it
	void prepareData(float* firstM, float* secondM, float* thirdM, float* psiFactor, float* raster)
	{
		double psi;

		spec.prepare(controls);
		spec.outputResults(firstM, secondM, thirdM, psi, raster);
		*psiFactor = sqrtf((psi - 1) / psi);
	}
};

#endif // !RENDER_H_


