/*
 * RotMatrix3D.h
 *
 *  Created on: 01.03.2016
 *      Author: Pingu
 */



#ifndef ROTMATRIX3D_H_
#define ROTMATRIX3D_H_
#pragma once

#include <iostream>
#include <math.h>

typedef double Vec3D[3];
typedef double Vec2D[2];
typedef double Matrix33[3][3];

using namespace std;

class RotMatrix3D
{
	Matrix33 mat;
public:
	//Destructor
	~RotMatrix3D() {}

	//Default constructor
	RotMatrix3D(): mat() {}

	//Initializes a matrix that contains the inputs as columns
	RotMatrix3D(Vec3D & column1, Vec3D & column2, Vec3D & column3):mat()
	{
		for(int i = 0; i < 3; i++)
		{
			mat[i][0] = column1[i];
			mat[i][1] = column2[i];
			mat[i][2] = column3[i];
		}
	}

	//Copy constructor
	RotMatrix3D(const RotMatrix3D & copy):mat()
	{
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				mat[i][j] = copy.mat[i][j];
	}

	//Assignment
	void operator=(const RotMatrix3D & copy)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				mat[i][j] = copy.mat[i][j];
	}

	//Saves A*input to target
	void transform(Vec3D & input, Vec3D & target)
	{
		target[0] = mat[0][0]*input[0] + mat[0][1]*input[1] +mat[0][2]*input[2];
		target[1] = mat[1][0]*input[0] + mat[1][1]*input[1] +mat[1][2]*input[2];
		target[2] = mat[2][0]*input[0] + mat[2][1]*input[1] +mat[2][2]*input[2];
	}

	//Saves A^T*input to target
	void invTransform(Vec3D & input, Vec3D & target)
	{
		target[0] = mat[0][0]*input[0] + mat[1][0]*input[1] +mat[2][0]*input[2];
		target[1] = mat[0][1]*input[0] + mat[1][1]*input[1] +mat[2][1]*input[2];
		target[2] = mat[0][2]*input[0] + mat[1][2]*input[1] +mat[2][2]*input[2];
	}

	//Permanently transposes the Matrix
	void transpose()
	{
		double dummy;
		dummy = mat[0][1];
		mat[0][1] = mat[1][0];
		mat[1][0] = dummy;

		dummy = mat[2][1];
		mat[2][1] = mat[1][2];
		mat[1][2] = dummy;

		dummy = mat[0][2];
		mat[0][2] = mat[2][0];
		mat[2][0] = dummy;
	}

	RotMatrix3D multiply(const RotMatrix3D & right)
	{
		RotMatrix3D result = RotMatrix3D();
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.mat[i][j] = mat[i][0] * right.mat[0][j] + mat[i][1] * right.mat[1][j]
				+ mat[i][2] * right.mat[2][j];
		return result;
	}

	void multiplyOnto(RotMatrix3D & right)
	{
		*this = this->multiply(right);
	}

	//deletes any numbers that are very small
	void clearArtifacts()
	{
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				if(fabs((mat[i][j])) < 1e-10)
					mat[i][j] = 0;
	}

	void setToIdentity()
	{
		mat[0][0] = mat[1][1] = mat[2][2] = 1;
		mat[1][0] = mat[2][0] = mat[0][1] = mat[0][2] = mat[1][2] = mat[2][1] = 0;
	}

	//Prints the matrix
	void print(ostream& os)
	{
		for(int i = 0; i < 3; i++)
			os << mat[i][0] << ' ' << mat[i][1] << ' ' << mat[i][2] << endl;
	}

	void toFloatArray(float* in)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				in[i * 3 + j] = mat[i][j];
	}
};



#endif /* ROTMATRIX3D_H_ */
