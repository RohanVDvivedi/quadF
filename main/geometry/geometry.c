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
	return sqrt(dot(A, A));
}

void multiply(quaternion* C, quaternion* A, quaternion* B)
{
	C->sc = (A->sc * B->sc) - (A->xi * B->xi) - (A->yj * B->yj) - (A->zk * B->zk);
	C->xi = (A->sc * B->xi) + (A->xi * B->sc) + (A->yj * B->zk) - (A->zk * B->yj);
	C->yj = (A->sc * B->yj) - (A->xi * B->zk) + (A->yj * B->sc) + (A->zk * B->xi);
	C->zk = (A->sc * B->zk) + (A->xi * B->yj) - (A->yj * B->xi) + (A->zk * B->sc);
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

void conjugate(quaternion* destination, quaternion* source)
{
	destination->sc = +(source->sc);
	destination->xi = -(source->xi);
	destination->yj = -(source->yj);
	destination->zk = -(source->zk);
}

// F is the final vector we get by rotating I by a quaternion R
// F = R * I * Rconj
void rotate_vector(vector* F, quaternion* R, vector* I)
{
	quaternion Rconj;
	conjugate(&Rconj, R);

	quaternion Itemp;
	Itemp.sc = 0;
	Itemp.xi = I->xi;
	Itemp.yj = I->yj;
	Itemp.zk = I->zk;

	quaternion temp;

	quaternion Ftemp;

	multiply(&temp, 	R, &Itemp			);
	multiply(&Ftemp, 	&temp, 		&Rconj	);

	F->xi = Ftemp.xi;
	F->yj = Ftemp.yj;
	F->zk = Ftemp.zk;
}