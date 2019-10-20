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

void unit_vector(vector* unitResult, vector* A);

// C = component of A parallel to B
void parallel_component(vector* C, vector* A, vector* B);

// C = component of A perpendicular to B
void perpendicular_component(vector* C, vector* A, vector* B);

// get magnitude of the vector
double magnitude_vector(vector* D);

// get magnitude of the quaternion
double norm(quaternion* D);

// multiply quaternions with hamilton product in order as C = AB
void hamilton_product(quaternion* C, quaternion* A, quaternion* B);

void to_quaternion(quaternion* destination, quat_raw* source);

void get_unit_rotation_axis(vector* axis, quaternion* source);

void conjugate(quaternion* destination, quaternion* source);

void reciprocal(quaternion* destination, quaternion* source);

// rotates a given vector by a given quaternion, must return 0
double rotate_vector(vector* result, quaternion* rotation_quaternion, vector* initial);

// Spherical linear interpolation of two quaternions
void slerp_quaternion(quaternion* Result, quaternion* A, double factorA, quaternion* B);

// this method returns the quaternion rotation that caused tips of Ai and Bi non collinear vectors
// of the object, to move to final positions Af and Bf respectively
void get_quaternion_from_vectors_changes(quaternion* quat, vector* Af, vector* Ai, vector* Bf, vector* Bi);

#endif