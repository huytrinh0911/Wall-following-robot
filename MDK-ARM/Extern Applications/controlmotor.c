#include "controlmotor.h"


// Motors control
void Left_forward(void)
	{
		HAL_GPIO_WritePin(Left_control_1_GPIO_Port, Left_control_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Left_control_2_GPIO_Port, Left_control_2_Pin, GPIO_PIN_RESET);
	}
	
void Left_backward(void)
	{
		HAL_GPIO_WritePin(Left_control_1_GPIO_Port, Left_control_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Left_control_2_GPIO_Port, Left_control_2_Pin, GPIO_PIN_SET);
	}
	
void Left_stop(void)
	{
		HAL_GPIO_WritePin(Left_control_1_GPIO_Port, Left_control_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Left_control_2_GPIO_Port, Left_control_2_Pin, GPIO_PIN_RESET);
	}
	
void Right_forward(void)
	{
		HAL_GPIO_WritePin(Right_control_1_GPIO_Port, Right_control_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Right_control_2_GPIO_Port, Right_control_2_Pin, GPIO_PIN_SET);
	}
	
void Right_backward(void)
	{
		HAL_GPIO_WritePin(Right_control_1_GPIO_Port, Right_control_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Right_control_2_GPIO_Port, Right_control_2_Pin, GPIO_PIN_RESET);
	}
	
void Right_stop(void)
	{
		HAL_GPIO_WritePin(Right_control_1_GPIO_Port, Right_control_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Right_control_2_GPIO_Port, Right_control_2_Pin, GPIO_PIN_RESET);
	}
	


	