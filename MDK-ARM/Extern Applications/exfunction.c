#include "exfunction.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

extern float omegaR;
extern float omegaL;

extern uint8_t omegaR_buff[sizeof(float)];
extern uint8_t omegaL_buff[sizeof(float)];

extern float RightFront_dis;
extern float RightBack_dis;

uint8_t data_buff[sizeof(float)];

uint16_t cap_delay01 = 0;
uint16_t cap_delay02 = 0;
volatile uint16_t diff = 0;

void delay_us(uint16_t us)
{			
	
//	cap_delay01 = __HAL_TIM_GET_COUNTER(&htim1);
//	while(diff < us)
//	{
//		cap_delay02 = __HAL_TIM_GET_COUNTER(&htim1);
//		if(cap_delay02 >= cap_delay01)
//		{
//			diff = cap_delay02 - cap_delay01;
//		}
//		else if (cap_delay01 > cap_delay02)
//		{
//			diff = (49999 - cap_delay01) + cap_delay02;
//		}
//	}
//	diff = 0;
	
	

	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < us);
	
}

void SendToMATLAB_omegaR(void)
{
	memcpy(omegaR_buff, &omegaR, sizeof(float));
	HAL_UART_Transmit(&huart2, omegaR_buff, sizeof(float), HAL_MAX_DELAY);
//	HAL_UART_Transmit_DMA(&huart2, omegaR_buff, sizeof(float));
}


void SendToMATLAB_omegaL(void)
{
	memcpy(omegaL_buff, &omegaL, sizeof(float));
	HAL_UART_Transmit(&huart2, omegaL_buff, sizeof(float), HAL_MAX_DELAY);
//	HAL_UART_Transmit_DMA(&huart2, omegaR_buff, sizeof(float));
}

void SendToMATLAB(float *data)
{
	memcpy(data_buff, data, sizeof(float));
	HAL_UART_Transmit(&huart2, data_buff, sizeof(float), HAL_MAX_DELAY);
}






