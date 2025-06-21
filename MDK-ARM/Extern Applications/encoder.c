#include "encoder.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;



// Encoder Variable
extern volatile const float Sample_time_ms;
extern const float PI;

extern volatile uint8_t isChange_R;
extern volatile uint8_t isChange_L;

extern volatile uint16_t pre_encR_cnt;
extern volatile float preR;
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

extern float const omegaR_SP;

extern uint16_t test;
// ---------------------------------------------------------------------------------


// Encoders processing
void Encoder_processing_ver0(void)
{
	encR_cnt = (__HAL_TIM_GET_COUNTER(&htim2));
	encL_cnt = (__HAL_TIM_GET_COUNTER(&htim3));
	
	rpsL = ((1000/(2800*(Sample_time_ms)))*((encL_cnt-0x8000)));
	rpsR = ((1000/(2800*(Sample_time_ms)))*((encR_cnt-0x8000)));
	
	__HAL_TIM_SET_COUNTER(&htim2, 0x8000);
	__HAL_TIM_SET_COUNTER(&htim3, 0x8000);

//	rpmL = 60*rpsL;
//	rpmR = 60*rpsR;
	omegaL = rpsL*2*PI;
	omegaR = rpsR*2*PI;
}
	
void Encoder_processing_ver1(void)
{
	if(isChange_R)
	{
		encR_cnt = (__HAL_TIM_GET_COUNTER(&htim2));
		rpsR = ((1000/(2800*(Sample_time_ms)))*((encR_cnt-0x8000)));
	}
	else
	{
		isChange_R = 0;
		rpsR = 0;
	}
	
	if(isChange_L)
	{
		encL_cnt = (__HAL_TIM_GET_COUNTER(&htim3));
		rpsL = ((1000/(2800*(Sample_time_ms)))*((encL_cnt-0x8000)));
	}
	else
	{
		isChange_L = 0;
		rpsL = 0;
	}
	
	__HAL_TIM_SET_COUNTER(&htim2, 0x8000);
	__HAL_TIM_SET_COUNTER(&htim3, 0x8000);

//	rpmL = 60*rpsL;
//	rpmR = 60*rpsR;
	omegaL = rpsL*2*PI;
	omegaR = rpsR*2*PI;
}


void Encoder_processing_ver2(void)
{
	rpsL = ((1000/(2800*(Sample_time_ms)))*((pre_encL_cnt + encL_cnt)/2));
	rpsR = ((1000/(2800*(Sample_time_ms)))*((pre_encR_cnt + encR_cnt)/2));

	rpmL = 60*rpsL;
	rpmR = 60*rpsR;
	omegaL = rpsL*2*PI;
	omegaR = rpsR*2*PI;
	
	pre_encL_cnt = encL_cnt;
	pre_encR_cnt = encR_cnt;
}


void Encoder_processing_ver3(void)
{
	HAL_DMA_Start(&hdma_memtomem_dma1_channel1, (uint32_t)&TIM2->CNT, (uint32_t)&encR_cnt, sizeof(uint16_t));
		
	rpsL = ((1000/(2800*(Sample_time_ms)))*((pre_encL_cnt + encL_cnt)/2));
//	rpsR = ((1000/(2800*(Sample_time_ms)))*(((encR_cnt-0x8000)+(pre_encR_cnt-0x8000))/2));
	rpsR = ((1000/(2800*(Sample_time_ms)))*(encR_cnt-0x8000));
//	rpsR = (((1000/(2800*(Sample_time_ms)))*(encR_cnt-0x8000))+rpsR)/2;
	
	__HAL_TIM_SET_COUNTER(&htim2, 0x8000);
	__HAL_TIM_SET_COUNTER(&htim3, 0x8000);

	rpmL = 60*rpsL;
	rpmR = 60*rpsR;
	omegaL = rpsL*2*PI;
	omegaR = rpsR*2*PI;
	
//	pre_encL_cnt = encL_cnt;
	pre_encR_cnt = encR_cnt;
//	preR = encR_cnt-0x8000;
}

void Encoder_processing_ver4(void)
{
	encR_cnt = (__HAL_TIM_GET_COUNTER(&htim2));
	encL_cnt = (__HAL_TIM_GET_COUNTER(&htim3));
	
	
	rpsL = ((1000/(2800*(Sample_time_ms)))*((pre_encL_cnt + encL_cnt)/2));
//	rpsR = ((1000/(2800*(Sample_time_ms)))*(((encR_cnt-0x8000)+(pre_encR_cnt-0x8000))/2));
//	rpsR = ((1000/(2800*(Sample_time_ms)))*(encR_cnt-0x8000));
	rpsR = (((1000/(2800*(Sample_time_ms)))*(encR_cnt-0x8000))+rpsR)/2;
	
	__HAL_TIM_SET_COUNTER(&htim2, 0x8000);
	__HAL_TIM_SET_COUNTER(&htim3, 0x8000);

	rpmL = 60*rpsL;
	rpmR = 60*rpsR;
	omegaL = rpsL*2*PI;
	omegaR = rpsR*2*PI;
	
//	pre_encL_cnt = encL_cnt;
	pre_encR_cnt = encR_cnt;
//	preR = encR_cnt-0x8000;
}



