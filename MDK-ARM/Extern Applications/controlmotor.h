#ifndef controlmotor
#define controlmotor

#include "main.h"
#include "encoder.h"
#include "takesample.h"
#include "exfunction.h"
#include "ultrasonic.h"
#include "pid.h"
#include "pwm.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal_tim.h"


/* USER CODE BEGIN PFP */

void Left_forward(void);
void Left_backward(void);
void Left_stop(void);
	
void Right_forward(void);
void Right_backward(void);
void Right_stop(void);


/* USER CODE END PFP */


#endif


