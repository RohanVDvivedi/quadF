#include<motor_manager.h>

void write_corrections_to_motors(corrections* corr)
{
	int left_front  = (int)(corr->altitude_corr + corr->pitch_corr - corr->roll_corr);
	int right_front = (int)(corr->altitude_corr + corr->pitch_corr + corr->roll_corr);
	int left_back   = (int)(corr->altitude_corr - corr->pitch_corr - corr->roll_corr);
	int right_back  = (int)(corr->altitude_corr - corr->pitch_corr + corr->roll_corr);

	unsigned int lf,rf,lb,rb;
	lf = left_front < 0 ? 0 : left_front;
	rf = right_front < 0 ? 0 : right_front;
	lb = left_back < 0 ? 0 : left_back;
	rb = right_back < 0 ? 0 : right_back;

	write_values_bldc(lf, rf, lb, rb);
}