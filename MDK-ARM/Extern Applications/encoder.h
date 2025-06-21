#ifndef encoder
#define encoder

#include "main.h"
#include "exfunction.h"
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

void Encoder_processing_ver0(void);
void Encoder_processing_ver1(void);
void Encoder_processing_ver2(void);
void Encoder_processing_ver3(void);
void Encoder_processing_ver4(void);
void Encoder_processing_ver5(void);



/* USER CODE END PFP */


#endif
