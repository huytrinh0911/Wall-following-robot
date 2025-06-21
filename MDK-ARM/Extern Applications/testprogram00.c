#include "testprogram00.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern volatile uint16_t pre_encR_cnt;
extern volatile uint16_t encR_cnt_PA0_PA1;
extern volatile uint16_t encR_cnt;

extern volatile uint16_t pre_encL_cnt;
extern volatile uint16_t encL_cnt;
extern volatile uint16_t encL_cnt_PA6_PA7;

extern uint16_t pwm_setup01;
extern uint16_t pwm_setup02;

// ---------------------------------------------------------------------------------


void setup00(void)
{
	Right_stop();
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	
	Left_stop();
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
}

void setup01(void)
{
	Right_forward();
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_setup01);
	
	Left_forward();
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm_setup01);
}

void setup02(void)
{
	Right_backward();
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_setup02);
	
	Left_backward();
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm_setup02);
}

void test_enc_R(void)
{
	encR_cnt = (__HAL_TIM_GET_COUNTER(&htim2));
}

void test_enc_L(void)
{
	encL_cnt = (__HAL_TIM_GET_COUNTER(&htim3));
}


