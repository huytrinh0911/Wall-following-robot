#include "takesample.h"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
//extern IWDG_HandleTypeDef hiwdg;


// General Flag/Variable
extern volatile uint8_t Sample_flag;
extern volatile uint8_t Sample_enc_flag;

extern volatile const float Sample_time_ms;
extern float const PI;
extern uint16_t test;
// ---------------------------------------------------------------------------------


extern volatile uint8_t isChange_R;
extern volatile uint8_t isChange_L;

extern volatile uint16_t pre_encR_cnt;
extern volatile uint16_t encR_cnt_PA0_PA1;
extern volatile uint16_t encR_cnt;
extern volatile int16_t encR_cnt_0;
extern volatile int16_t encR_cnt_1;

extern volatile uint16_t pre_encL_cnt;
extern volatile uint16_t encL_cnt_PA6_PA7;
extern volatile uint16_t encL_cnt;
extern volatile int16_t encL_cnt_0;
extern volatile int16_t encL_cnt_1;

volatile extern float omegaR;
volatile extern float omegaL;
extern float rpmR;
extern float rpmL;
extern float rpsR;
extern float rpsL;

volatile extern float omegaR_SP;
volatile extern float omegaL_SP;

extern float PID_R;
extern float PID_L;

volatile uint8_t sample_order = 0;

extern float PID_dis;
extern float dis;
extern float distance_setpoint;
extern float omega;

extern float dis;
extern float omega;
extern float theta;
extern float RightFront_dis;
extern float RightBack_dis;
extern float Front_dis;

volatile extern uint8_t Turn_Left;
volatile extern uint8_t Turn_Right_RF;
volatile extern uint8_t Turn_Right_RB;
volatile extern uint8_t Turn_Right;


// ---------------------------------------------------------------------------------

void sample_enc_ver0(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;	
		Encoder_processing_ver0();
			
		SendToMATLAB_omegaR();
	}
}	

void sample_enc_ver1(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		
		if(!sample_order)
		{
			sample_order++;
			encR_cnt_0 = (__HAL_TIM_GET_COUNTER(&htim2));
			encL_cnt_0 = (__HAL_TIM_GET_COUNTER(&htim3));
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			__HAL_TIM_SET_COUNTER(&htim3, 0);
		}
		else
		{
			sample_order = 0;
			encR_cnt_1 = (__HAL_TIM_GET_COUNTER(&htim2));
			encL_cnt_1 = (__HAL_TIM_GET_COUNTER(&htim3));
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			
//			encR_cnt = (encR_cnt_0 + encR_cnt_1)/2;
			Encoder_processing_ver1();
			SendToMATLAB_omegaR();
		}
	}
}

void sample_enc_ver2(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		
		encR_cnt = (__HAL_TIM_GET_COUNTER(&htim2));
		encL_cnt = (__HAL_TIM_GET_COUNTER(&htim3));
		
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		
		Encoder_processing_ver2();
		
		SendToMATLAB_omegaR();
	}
}

void sample_enc_ver3(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		Encoder_processing_ver3();
		SendToMATLAB_omegaR();
	}
}

void sample_speedctrl_ver0(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		Encoder_processing_ver4();
		SendToMATLAB_omegaR();
//		PID_R = PID_R_control(0, omegaR);
		PID_R = PID_R_control(omegaR_SP, omegaR);
		PIDtoPWM_R_ver1(PID_R);
	}
}

void sample_speedctrl_ver1(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		Encoder_processing_ver0();
		SendToMATLAB_omegaR();
		PID_R = PID_R_control(omegaR_SP, omegaR);
		PIDtoPWM_R_ver1(PID_R);
	}
}

void sample_speedctrl_ver2(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		Encoder_processing_ver1();
//		PID_R = PID_R_control(omegaR_SP, omegaR);
		PID_R = PID_R_control(5, omegaR);
		PID_L = PID_L_control(omegaL_SP, omegaL);
		PIDtoPWM_R_ver1(PID_R);
		PIDtoPWM_L_ver1(PID_L);
		
		SendToMATLAB_omegaR();
	}
}

void sample_all_ver0(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		Encoder_processing_ver1();
////		
		PID_R = PID_R_control(omegaR_SP, omegaR);
		PID_L = PID_L_control(omegaL_SP, omegaL);
////		
//		PID_R = PID_R_control(5, omegaR);
//		PID_L = PID_L_control(6, omegaL);
////		
		PIDtoPWM_R_ver1(PID_R);
		PIDtoPWM_L_ver1(PID_L);
		
		Ultrasonic_getData();
		PID_dis = PID_distance(distance_setpoint, dis);
		PIDtoOmega(PID_dis);
//		
//		SendToMATLAB_omegaR();
//		SendToMATLAB(&Front_dis);
//		HAL_IWDG_Refresh(&hiwdg);
	}
}

void sample_all_ver1(void)
{
	if(Sample_flag)
	{
		Sample_flag = 0;
		Encoder_processing_ver1();
		
		PID_R = PID_R_control(omegaR_SP, omegaR);
		PID_L = PID_L_control(omegaL_SP, omegaL);
		
//		PID_R = PID_R_control(5, omegaR);
//		PID_L = PID_L_control(5, omegaL);
		
		PIDtoPWM_R_ver1(PID_R);
		PIDtoPWM_L_ver1(PID_L);
		
		Ultrasonic_getData();
		if(Turn_Left)
		{
			PID_dis = 350;
		}
		else
		{
			PID_dis = PID_distance(distance_setpoint, dis);
		}
		
		PIDtoOmega(PID_dis);
		
//		SendToMATLAB_omegaR();
//		SendToMATLAB(&omegaL);
	}
}


void sample_all_ver2(void)
{
	if(Sample_flag) // 200ms
	{
		Sample_flag = 0;
		Ultrasonic_getData();
//		PID_dis = PID_distance(distance_setpoint, dis);
//		PIDtoOmega(PID_dis);
	}
	
	if(Sample_enc_flag) // 50ms
	{
		Sample_enc_flag = 0;
		Encoder_processing_ver1();
//		
////		PID_R = PID_R_control(omegaR_SP, omegaR);
////		PID_L = PID_L_control(omegaL_SP, omegaL);
////		
		PID_R = PID_R_control(0, omegaR);
		PID_L = PID_L_control(0, omegaL);
////		
		PIDtoPWM_R_ver1(PID_R);
		PIDtoPWM_L_ver1(PID_L);
		

//		
//		SendToMATLAB_omegaR();
//		SendToMATLAB(&Front_dis);
//		HAL_IWDG_Refresh(&hiwdg);
	}
}

void sample_all_ver3(void)
{
	if(Sample_flag) // 100ms
	{
		Sample_flag = 0;
		Ultrasonic_getData();
		PID_dis = PID_distance(distance_setpoint, dis);
		PIDtoOmega(PID_dis);
	}
}


extern float r;
extern float b;
extern float v_setpoint;


void sample_all_ver4(void)
{
	if(Sample_flag) // 100ms
	{
		Sample_flag = 0;
		Ultrasonic_getData();
		if(Turn_Left)
		{
//			PIDtoOmega(400);
			omega = PI/2;
			omegaR_SP = (1/r)*v_setpoint + 1.5*(b/r)*omega;
			omegaL_SP = (1/r)*v_setpoint - 2.25*(b/r)*omega;
		}
		else if(Turn_Right_RF && Turn_Right_RB)
		{
			omega = -PI/2;
			omegaR_SP = (1/r)*v_setpoint + 2*(b/r)*omega;
//			omegaR_SP=0;
			omegaL_SP = (1/r)*v_setpoint - 1.5*(b/r)*omega;
		}
		
		else if(!Turn_Left)
		{
			PID_dis = PID_distance(distance_setpoint, dis);
			PIDtoOmega(PID_dis);
		}
		
	}
}

