#ifndef pid
#define pid

#include "main.h"
#include "encoder.h"
#include "takesample.h"
#include "exfunction.h"
#include "ultrasonic.h"
#include "pwm.h"
#include "controlmotor.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal_tim.h"


/* USER CODE BEGIN PFP */

float PID_R_control(float setpoint, float measure);
float PID_L_control(float setpoint, float measure);
float PID_distance(float setpoint, float measure);
void PIDtoOmega(float PID_dis);




/* USER CODE END PFP */


#endif
