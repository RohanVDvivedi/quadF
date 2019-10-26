#include<motor_manager.h>

void write_corrections_to_motors(corrections* corr)
{
	unsigned int left_front  = (unsigned int)(corr->altitude_corr);
	unsigned int right_front = (unsigned int)(corr->altitude_corr);
	unsigned int left_back   = (unsigned int)(corr->altitude_corr);
	unsigned int right_back  = (unsigned int)(corr->altitude_corr);
	write_values_bldc(left_front, right_front, left_back, right_back);
}