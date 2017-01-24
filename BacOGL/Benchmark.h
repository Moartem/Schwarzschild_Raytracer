#ifndef BENCHMARK_H_
#define BENCHMARK_H_
#pragma once
 
#include <omp.h>
#include <string>
#include <ostream>

class Benchmark
{
	double startTime;
	double maxDuration;
	double accumulatedTime;
	unsigned int calls;
	std::string name;
public:
	Benchmark(std::string inName): name(inName), startTime(0), 
		accumulatedTime(0), calls(0), maxDuration(0) {}
	
	void startMeasurement()
	{
		startTime = omp_get_wtime();
	}

	void stopMeasurement()
	{
		double time = omp_get_wtime() - startTime;
		maxDuration = maxDuration < time ? time : maxDuration;
		accumulatedTime += time;
		calls++;
	}

	void printReport(std::ostream& os)
	{
		os << "Benchmark " << name << " report:" << std::endl;
		os << "Maximum: " << maxDuration << std::endl;
		os << "Average: " << accumulatedTime / calls << std::endl;
		os << "Number of calls:" << calls << std::endl;
	}

};


#endif //! BENCHMARK_H_

