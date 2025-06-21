#ifndef takesample
#define takesample

#include "main.h"
#include "encoder.h"
#include "ultrasonic.h"
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

void sample_enc_ver0(void);
void sample_enc_ver1(void);
void sample_enc_ver2(void);
void sample_enc_ver3(void);
void sample_enc_ver4(void);
void sample_enc_ver5(void);

void sample_speedctrl_ver0(void);
void sample_speedctrl_ver1(void);
void sample_speedctrl_ver2(void);
void sample_speedctrl_ver3(void);
void sample_speedctrl_ver4(void);
void sample_speedctrl_ver5(void);
void sample_speedctrl_ver6(void);

void sample_all_ver0(void);
void sample_all_ver1(void);
void sample_all_ver2(void);
void sample_all_ver3(void);
void sample_all_ver4(void);
void sample_all_ver5(void);



/* USER CODE END PFP */


#endif
