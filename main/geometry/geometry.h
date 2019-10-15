#ifndef GEOMETRY_H
#define GEOMETRY_H

#include<math.h>

typedef struct vector vector;
struct vector
{
	double xi;
	double yj;
	double zk;
};

typedef struct quaternion quaternion;
struct quaternion
{
	// scalar component
	double sc;

	// i, j, and k component
	double xi;
	double yj;
	double zk;
};

typedef struct quat_raw quat_raw;
struct quat_raw
{
	// the rotation amount in angle degrees
	double theta;

	// vector about which rotation is done
	vector vectr;
};

// C = A + B
void sum(vector* C, vector* A, vector* B);

// C = A - B
void diff(vector* C, vector* A, vector* B);

// c = A * sc
void multiply_scalar(vector* C, vector* A, double sc);

// C = A X B
void cross(vector* C, vector* A, vector* B);

// A.B
double dot(vector* A, vector* B);

// angle between vectors in degrees
double angle_between_vectors(vector* A, vector* B);

// C = component of A parallel to B
void parallel_component(vector* C, vector* A, vector* B);

// C = component of A perpendicular to B
void perpendicular_component(vector* C, vector* A, vector* B);

// get magnitude of the vector
double magnitude_vector(vector* D);

// get magnitude of the quaternion
double magnitude_quaternion(quaternion* D);

// multiply quaternions
void multiply(quaternion* C, quaternion* A, quaternion* B);

void to_quaternion(quaternion* destination, quat_raw* source);

void conjugate(quaternion* destination, quaternion* source);

// rotates a given vector by a given quaternion
void rotate_vector(vector* result, quaternion* rotation_quaternion, vector* initial);

#endif