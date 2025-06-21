#include "pwm.h"

extern TIM_HandleTypeDef htim4;

float fPWM_R;
float fPWM_L;

void PIDtoPWM_R_ver0(float PID_R)
{
	if(PID_R > 0)
	{
		Right_forward();
		fPWM_R = 210 + PID_R*0.47368421; //fPWM = ((PID)*((189)/(399)))+210;
	}
	else if (PID_R < 0)
	{
		Right_backward();
		fPWM_R = 210 + (-PID_R)*0.47368421; // fPWM = ((-PID)*((189)/(399)))+210;
	}
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(uint16_t)fPWM_R);
}


void PIDtoPWM_R_ver1(float PID_R)
{
	if(PID_R > 0)
	{
		Right_forward();
		fPWM_R = 210 + PID_R*0.47368421; //fPWM = ((PID)*((189)/(399)))+210;
	}
	else if (PID_R < 0)
	{
		Right_forward();
//		fPWM_R = 210;
		fPWM_R = 210 + PID_R*0.47368421;
	}
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(uint16_t)fPWM_R);
}


void PIDtoPWM_L_ver1(float PID_L)
{
	if(PID_L > 0)
	{
		Left_forward();
		fPWM_L = 210 + PID_L*0.47368421; //fPWM = ((PID)*((189)/(399)))+210;
	}
	else if (PID_L < 0)
	{
		Left_forward();
//		fPWM_L = 210;
		fPWM_L = 210 + PID_L*0.47368421;
	}
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(uint16_t)fPWM_R);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,(uint16_t)fPWM_L);
}

