#ifndef exfunction
#define exfunction

#include "main.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "takesample.h"
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

void delay_us(uint16_t us);
void SendToMATLAB_omegaR(void);
void SendToMATLAB_omegaL(void);
void SendToMATLAB(float *data);



/* USER CODE END PFP */


#endif
