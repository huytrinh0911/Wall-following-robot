#ifndef testprogram00
#define testprogram00

#include "main.h"
#include "encoder.h"
#include "takesample.h"
#include "exfunction.h"
#include "ultrasonic.h"
#include "pid.h"
#include "controlmotor.h"
#include "pwm.h"


#include <math.h>
#include <stdint.h>
#include <string.h>

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal_tim.h"


/* USER CODE BEGIN PFP */

void setup00(void);
void setup01(void);
void setup02(void);
void setup03(void);
void setup04(void);
void setup05(void);


void test_enc_R(void);
void test_enc_L(void);

/* USER CODE END PFP */


#endif

