#include<state.h>

state State = {
    .orientation = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0},
    .angular_velocity_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .acceleration_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .altitude = -1,
    .altitude_rate = 0.0,
};

void get_current_local_X_axis(vector* xl)
{
	vector X = {.xi = 1.0, .yj = 0.0, .zk = 0.0};
	rotate_vector(xl, &(State.orientation), &X);
}

void get_current_local_Y_axis(vector* yl)
{
	vector Y = {.xi = 0.0, .yj = 1.0, .zk = 0.0};
	rotate_vector(yl, &(State.orientation), &Y);
}

void get_current_local_Z_axis(vector* zl)
{
	vector Z = {.xi = 0.0, .yj = 0.0, .zk = 1.0};
	rotate_vector(zl, &(State.orientation), &Z);
}
#include<stdio.h>
void get_absolute_rotation_angles_about_local_axis(vector* angles)
{
	// get unit vectors about local axis, wrt to global axis
	// the global axis is the initial position of the sensor board
	vector xl;	get_current_local_X_axis(&xl);
	vector yl;	get_current_local_Y_axis(&yl);
	vector zl;	get_current_local_Z_axis(&zl);

	//printf("xl : %lf \t %lf \t %lf\n", xl.xi, xl.yj, xl.zk);
    //printf("yl : %lf \t %lf \t %lf\n", yl.xi, yl.yj, yl.zk);
    //printf("zl : %lf \t %lf \t %lf\n", zl.xi, zl.yj, zl.zk);

	double mag;
	vector zero_v = {.xi = 0.0, .yj = 0.0, .zk = 0.0};
	vector temp   = zero_v;
	vector temp1  = zero_v;

	// ABSOLUTE roll calculation
	// angles.xi = absolute roll
	temp = zero_v;
	mag = sqrt(xl.xi * xl.xi + xl.yj * xl.yj);
	if(mag != 0)
	{
		temp.xi = xl.yj / mag;
		temp.yj = -xl.xi / mag;
		cross(&temp1, &temp, &yl);
		if(angle_between_vectors(&temp1, &xl) > 170)
		{
			multiply_scalar(&temp, &temp, -1);
		}
		angles->xi = angle_between_vectors(&temp, &yl);
		if(yl.zk > 0 && zl.zk > 0)
		{
			angles->xi = angles->xi;
		}
		else if(yl.zk < 0 && zl.zk < 0)
		{
			angles->xi = angles->xi - 180;
		}
		else if(yl.zk < 0 && zl.zk > 0)
		{
			angles->xi = angles->xi - 180;
		}
		else if(yl.zk > 0 && zl.zk < 0)
		{
			angles->xi = angles->xi;
		}
	}
	else
	{
		angles->xi = NAN;
	}

	// ABSOLUTE pitch calculation
	// angles.yj = absolute pitch
	temp = zero_v;
	mag = sqrt(yl.xi * yl.xi + yl.yj * yl.yj);
	if(mag != 0)
	{
		temp.xi = yl.yj / mag;
		temp.yj = -yl.xi / mag;
		cross(&temp1, &xl, &temp);
		if(angle_between_vectors(&temp1, &yl) > 170)
		{
			multiply_scalar(&temp, &temp, -1);
		}
		angles->yj = angle_between_vectors(&temp, &xl);
		if(xl.zk > 0 && zl.zk > 0)
		{
			angles->yj = angles->yj;
		}
		else if(xl.zk < 0 && zl.zk < 0)
		{
			angles->yj = angles->yj - 180;
		}
		else if(xl.zk < 0 && zl.zk > 0)
		{
			angles->yj = angles->yj - 180;
		}
		else if(xl.zk > 0 && zl.zk < 0)
		{
			angles->yj = angles->yj;
		}
	}
	else
	{
		angles->yj = NAN;
	}

	// ABSOLUTE yaw calculation
	// angles.zk = absolute yaw
	temp = zero_v;
}