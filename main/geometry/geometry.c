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
	double cosine = dot(A, B)/(magnitude_vector(A)*magnitude_vector(B));
	if(cosine >= 1.0)
	{
		return 0;
	}
	else if(cosine <= -1.0)
	{
		return 180;
	}
	else
	{
		return (acos(cosine) * 180) / M_PI;
	}
}

// C = component of A parallel to B
void parallel_component(vector* C, vector* A, vector* B)
{
	// this step makes C = unit vector in direction of B
	multiply_scalar(C, B, 1/magnitude_vector(B));

	// this is the magnitude of the component of A in direction of B
	double parallel_component_magnitude = dot(A, B) / magnitude_vector(B);

	// multiply magnitude and the direction
	multiply_scalar(C, C, parallel_component_magnitude);
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
	vector temp = *A;
	return sqrt(dot(A, &temp));
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
#include<stdio.h>
void get_quaternion_from_vectors_changes(quaternion* quat, vector* Af, vector* Ai, vector* Bf, vector* Bi)
{
	vector A;diff(&A, Af, Ai);
	vector B;diff(&B, Bf, Bi);

	double ybyz = - ( ( (A.zk * B.xi) - (A.xi * B.zk) ) / ( (A.yj * B.xi) - (A.xi * B.yj) ) );
	double xbyy = - ( ( (A.yj * B.zk) - (A.zk * B.yj) ) / ( (A.xi * B.zk) - (A.zk * B.xi) ) );
	double xbyz = - ( ( (A.zk * B.yj) - (A.yj * B.zk) ) / ( (A.xi * B.yj) - (A.yj * B.xi) ) );

	quat_raw raw;
	raw.vectr.zk = sqrt(1.0/(1.0 + (ybyz * ybyz) + (xbyz * xbyz)));
	raw.vectr.yj = sqrt((ybyz * ybyz) / (1.0 + ((ybyz * ybyz) * (1.0 + (xbyy * xbyy)))));
	raw.vectr.xi = sqrt((xbyy * xbyy * xbyz * xbyz)/((xbyy * xbyy) + (xbyz * xbyz) + (xbyy * xbyy * xbyz * xbyz)));

	if(xbyy < 0)
	{
		raw.vectr.yj = -raw.vectr.yj;
	}
	if(xbyz < 0)
	{
		raw.vectr.zk = -raw.vectr.zk;
	}

	// find vector components perpendicuilar to raw.vectr
	vector Aip; vector Afp;
	perpendicular_component(&Aip, Ai, &(raw.vectr));
	perpendicular_component(&Afp, Af, &(raw.vectr));
	vector Bip; vector Bfp;
	perpendicular_component(&Bip, Bi, &(raw.vectr));
	perpendicular_component(&Bfp, Bf, &(raw.vectr));

	// this is the vecotor in same or opposite direction of raw.vectr
	vector AipCrossAfp;
	cross(&AipCrossAfp, &Aip, &Afp);
	double angle_AipCrossAfp_raw = angle_between_vectors(&AipCrossAfp, &(raw.vectr));
	double raw_vectr_sign_inversion_required_a = angle_AipCrossAfp_raw > 170 ? -1 : 1;

	// this is the vecotor in same or opposite direction of raw.vectr
	vector BipCrossBfp;
	cross(&BipCrossBfp, &Bip, &Bfp);
	double angle_BipCrossBfp_raw = angle_between_vectors(&BipCrossBfp, &(raw.vectr));
	double raw_vectr_sign_inversion_required_b = angle_BipCrossBfp_raw > 170 ? -1 : 1;

	// the difference betwen the angle between final vetcor and rotation vecotr specifies which
	// of A or B vecotr to use in finding the final value od anle
	// closer this is to 0, more the accuracy, by taking the calculation from that vector
	double angle_raw_vectr_Af_90 = angle_between_vectors(Af, &(raw.vectr));
	double angle_raw_vectr_Bf_90 = angle_between_vectors(Bf, &(raw.vectr));
	angle_raw_vectr_Af_90 = (angle_raw_vectr_Af_90 > 90) ? (180 - angle_raw_vectr_Af_90) : angle_raw_vectr_Af_90;
	angle_raw_vectr_Bf_90 = (angle_raw_vectr_Bf_90 > 90) ? (180 - angle_raw_vectr_Bf_90) : angle_raw_vectr_Bf_90;
	angle_raw_vectr_Af_90 = 90 - angle_raw_vectr_Af_90;
	angle_raw_vectr_Bf_90 = 90 - angle_raw_vectr_Bf_90; 

	if(raw_vectr_sign_inversion_required_a == raw_vectr_sign_inversion_required_b)
	{
		multiply_scalar(&(raw.vectr), &(raw.vectr), raw_vectr_sign_inversion_required_a);
	}
	// else use the one whose angle with rotation axis is closer to 90
	else if(angle_raw_vectr_Af_90 < angle_raw_vectr_Bf_90)
	{
		multiply_scalar(&(raw.vectr), &(raw.vectr), raw_vectr_sign_inversion_required_a);
	}
	else
	{
		multiply_scalar(&(raw.vectr), &(raw.vectr), raw_vectr_sign_inversion_required_b);
	}

	printf("accl : %lf\t %lf\t %lf\n", Af->xi, Af->yj, Af->zk);
	printf("magn : %lf\t %lf\t %lf\n", Bf->xi, Bf->yj, Bf->zk);
	printf("axl : %lf\t %lf\t \t\t\t\t %lf\n\n", raw.vectr.xi, raw.vectr.yj, raw.vectr.zk);

	double angle_by_A = angle_between_vectors(&Afp, &Aip);
	double angle_by_B = angle_between_vectors(&Bfp, &Bip);

	//printf("%lf \t%lf\n\n", angle_AipCrossAfp_raw, angle_BipCrossBfp_raw);
	//printf("%lf \t%lf\n\n", angle_by_A, angle_by_B);

	raw.theta = (angle_by_A + angle_by_B)/2;

	to_quaternion(quat, &raw);
}