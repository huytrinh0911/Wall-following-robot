#ifndef ultrasonic
#define ultrasonic

#include "main.h"
#include "encoder.h"
#include "takesample.h"
#include "exfunction.h"
#include "pid.h"
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

void Ultrasonic_RightBack_Read(void); // channel 1
void Ultrasonic_RightFront_Read(void); // channel 2	
void Ultrasonic_Front_Read(void); // channel 3
void Ultrasonic_getData(void);


/* USER CODE END PFP */


#endif
