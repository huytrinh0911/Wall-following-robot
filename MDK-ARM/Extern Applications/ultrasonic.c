#include "ultrasonic.h"

extern TIM_HandleTypeDef htim1;
extern IWDG_HandleTypeDef hiwdg;



// Ultrasonic Variable
extern uint32_t InCapFR_01;
extern uint32_t InCapFR_02;
extern uint32_t InCapRFR_01;
extern uint32_t InCapRFR_02;
extern uint32_t InCapRBR_01;
extern uint32_t InCapRBR_02;

extern uint32_t diff_f;
extern uint32_t diff_rf;
extern uint32_t diff_rb;
extern uint8_t IsFirstCap;

extern float Front_dis;
extern float RightFront_dis;
extern float RightBack_dis;

extern volatile uint8_t F_done;
extern volatile uint8_t RF_done;
extern volatile uint8_t RB_done;

volatile extern uint8_t Front_close;
volatile extern uint8_t Front_distant;
volatile extern uint8_t Turn_Left;

volatile extern uint8_t Turn_Right_RF;
volatile extern uint8_t Turn_Right_RB;
volatile extern uint8_t Turn_Right;


extern float theta;
extern float dis;
// ---------------------------------------------------------------------------------

// Ultrasonic sensors
void Ultrasonic_RightFront_Read(void) // channel 2
{
	RF_done = 0;
	HAL_GPIO_WritePin(Ultrasonic_Trigger_RightFront_GPIO_Port, Ultrasonic_Trigger_RightFront_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
//	delay_us(30); // wait for 30 us
	HAL_Delay(1);
	HAL_GPIO_WritePin(Ultrasonic_Trigger_RightFront_GPIO_Port, Ultrasonic_Trigger_RightFront_Pin, GPIO_PIN_RESET); // pull the TRIG pin LOW
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
	while(!RF_done);
	if(RightFront_dis > 80)
	{
		Turn_Right_RF = 1;
	}
	else 
	{
		Turn_Right_RF = 0;
	}
}	

void Ultrasonic_RightBack_Read(void) // channel 1
{
	RB_done = 0;
	HAL_GPIO_WritePin(Ultrasonic_Trigger_RightBack_GPIO_Port, Ultrasonic_Trigger_RightBack_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
//	delay_us(30); // wait for 30 us
	HAL_Delay(1);
	HAL_GPIO_WritePin(Ultrasonic_Trigger_RightBack_GPIO_Port, Ultrasonic_Trigger_RightBack_Pin, GPIO_PIN_RESET); // pull the TRIG pin LOW
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	while(!RB_done);
	if(RightBack_dis > 80)
	{
		Turn_Right_RB = 1;
	}
	else 
	{
		Turn_Right_RB = 0;
	}
}
	
	
void Ultrasonic_Front_Read(void) // channel 3
{
	F_done = 0;
	HAL_GPIO_WritePin(Ultrasonic_Trigger_Front_GPIO_Port, Ultrasonic_Trigger_Front_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
//	delay_us(30); // wait for 30 us
	HAL_Delay(1);
	HAL_GPIO_WritePin(Ultrasonic_Trigger_Front_GPIO_Port, Ultrasonic_Trigger_Front_Pin, GPIO_PIN_RESET); // pull the TRIG pin LOW
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
	while(!F_done);

	
	if(Front_dis < 50)
	{
		Front_close++;
		if(Front_close > 1)
		{
			Front_close = 1;
			Front_distant = 0;
			Turn_Left = 1;
		}
	}
	else
	{
		Front_distant++;
		if(Front_distant > 1)
		{
			Front_distant = 1;
			Front_close = 0;
			Turn_Left = 0;
		}
	}

}
	
void Ultrasonic_getData(void)
{
	Ultrasonic_Front_Read();
	Ultrasonic_RightFront_Read();
	Ultrasonic_RightBack_Read();
	theta = atan((RightFront_dis - RightBack_dis)/6);
	dis = ((RightFront_dis + RightBack_dis)/2)*(cos(theta));
//	
	HAL_IWDG_Refresh(&hiwdg);
//	

}
	
