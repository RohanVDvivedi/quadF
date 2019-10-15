#include<geometry.h>

void sum(vector* C, vector* A, vector* B)
{
	C->xi = A->xi + B->xi;
	C->yj = A->yj + B->yj;
	C->zk = A->zk + B->zk;
}

void diff(vector* C, vector* A, vector* B)
{
	C->xi = A->xi - B->xi;
	C->yj = A->yj - B->yj;
	C->zk = A->zk - B->zk;
}

void multiply_scalar(vector* C, vector* A, double sc)
{
	C->xi = A->xi * sc;
	C->yj = A->yj * sc;
	C->zk = A->zk * sc;
}

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

double angle_between_vectors(vector* A, vector* B)
{
	return (acos(dot(A, B)/(magnitude_vector(A)*magnitude_vector(B))) * 180) / M_PI;
}

// C = component of A parallel to B
void parallel_component(vector* C, vector* A, vector* B)
{
	// this step makes C = unit vector in direction of A
	multiply_scalar(C, A, 1/magnitude_vector(A));

	// this is the magnitude of the component of A in direction of B
	double parallel_component_magnitude = dot(A, B) / magnitude_vector(B);

	multiply_scalar(C, A, parallel_component_magnitude);
}

// C = component of A perpendicular to B
void perpendicular_component(vector* C, vector* A, vector* B)
{
	// here we made c parallel componnet of A
	parallel_component(C, A, B);

	// C = A - component of A parallel to B = component of A perpendiculat to B
	diff(C, A, C);
}

double magnitude_vector(vector* A)
{
	return sqrt(dot(A, A));
}

double magnitude_quaternion(quaternion* A)
{
	return sqrt((A->sc * A->sc) + (A->xi * A->xi) + (A->yj * A->yj) + (A->zk * A->zk));
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
	double sine   = sin((((source->theta)*M_PI)/180.0) / 2);
	double cosine = cos((((source->theta)*M_PI)/180.0) / 2);
	double magnit = magnitude_vector(&(source->vectr));
	destination->sc = cosine;
	destination->xi = (sine * source->vectr.xi) / magnit;
	destination->yj = (sine * source->vectr.yj) / magnit;
	destination->zk = (sine * source->vectr.zk) / magnit;
}

void conjugate(quaternion* destination, quaternion* source)
{
	double magnit = magnitude_quaternion(source);
	destination->sc = (+(source->sc)) / magnit;
	destination->xi = (-(source->xi)) / magnit;
	destination->yj = (-(source->yj)) / magnit;
	destination->zk = (-(source->zk)) / magnit;
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