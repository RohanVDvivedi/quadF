#include<geometry.h>

void cross(vector* C, vector* A, vector* B)
{
	C->xi = (A->yj * B->zk) - (A->zk * B->yj);
	C->yj = (A->zk * B->xi) - (A->xi * B->zk);
	C->zk = (A->xi * B->yj) - (A->yj * B->xi);
}

double dot(vector* A, vector* B)
{
	return (A->xi * B->xi) + (A->yj * B->yj) + (A->zk * B->zk);
}

double magnitude(vector* A)
{
	return pow((A->xi * A->xi) + (A->yj * A->yj) + (A->zk * A->zk), 0.5);
}

void multiply(quaternion* C, quaternion* A, quaternion* B)
{

}

void to_quaternion(quaternion* destination, quat_raw* source)
{
	double sine   = sin((((source->theta)*180.0)/M_PI) / 2);
	double cosine = cos((((source->theta)*180.0)/M_PI) / 2);
	destination->sc = cosine;
	destination->xi = sine * source->vectr.xi;
	destination->yj = sine * source->vectr.xi;
	destination->zk = sine * source->vectr.xi;
}