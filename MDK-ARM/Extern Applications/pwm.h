#ifndef pwm
#define pwm

#include "main.h"
#include "encoder.h"
#include "takesample.h"
#include "exfunction.h"
#include "ultrasonic.h"
#include "pid.h"
#include "controlmotor.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal_tim.h"


/* USER CODE BEGIN PFP */

void PIDtoPWM_R_ver0(float PID_R);
void PIDtoPWM_R_ver1(float PID_R);
void PIDtoPWM_R_ver2(float PID_R);

void PIDtoPWM_L_ver0(float PID_R);
void PIDtoPWM_L_ver1(float PID_R);
void PIDtoPWM_L_ver2(float PID_R);


/* USER CODE END PFP */


#endif
