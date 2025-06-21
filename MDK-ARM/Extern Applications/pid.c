#include "pid.h"

extern volatile const float Sample_time_ms;
volatile const float sample_ultrasonic_time_ms = 100; 

// Right Motor PID Variable
extern float Kp_R;	// bat dau dao dong, Kp = 7.6 // 6
extern float Ki_R;
extern float Kd_R;

extern float uk_R;
extern float uk_1_R;
extern float ek_R;
extern float ek_1_R;
extern float ek_2_R;

extern float const umax_R;
extern float const umin_R;
extern float PID_R;
// ---------------------------------------------------------------------------------


// Left Motor PID Variable
extern float Kp_L;	// bat dau dao dong, Kp = 7.6 // 6
extern float Ki_L;
extern float Kd_L;

extern float uk_L;
extern float uk_1_L;
extern float ek_L;
extern float ek_1_L;
extern float ek_2_L;

extern float const umax_L;
extern float const umin_L;
extern float PID_L;
// ---------------------------------------------------------------------------------


// Distance PID Variable
extern float distance_setpoint;
	
extern float Kp_dis;	
extern float Ki_dis;
extern float Kd_dis;

extern float uk_dis;
extern float uk_1_dis;
extern float ek_dis;
extern float ek_1_dis;
extern float ek_2_dis;

extern float const umax_dis;
extern float const umin_dis;
extern float PID_dis;
extern float v_setpoint;

extern float omega;
volatile extern float omegaR_SP;
volatile extern float omegaL_SP;
extern float const PI;

float r = 2.15;
float b = 6;
// ---------------------------------------------------------------------------------



// PID
float PID_R_control(float setpoint, float measure)
{
	ek_2_R = ek_1_R;
	ek_1_R = ek_R;
	ek_R = setpoint - measure;
	uk_1_R = uk_R;
	uk_R = uk_1_R + Kp_R*(ek_R-ek_1_R) + Ki_R*((Sample_time_ms)/2000)*(ek_R+ek_1_R) + (Kd_R/((Sample_time_ms)/1000))*(ek_R-2*ek_1_R+ek_2_R);
	if(uk_R > umax_R)
	{
		uk_R = umax_R;
	}
	else if (uk_R < umin_R)
	{
		uk_R = umin_R;
	}
	return(uk_R);
}

float PID_L_control(float setpoint, float measure)
{
	ek_2_L = ek_1_L;
	ek_1_L = ek_L;
	ek_L = setpoint - measure;
	uk_1_L = uk_L;
	uk_L = uk_1_L + Kp_L*(ek_L-ek_1_L) + Ki_L*((Sample_time_ms)/2000)*(ek_L+ek_1_L) + (Kd_L/((Sample_time_ms)/1000))*(ek_L-2*ek_1_L+ek_2_L);
	if(uk_L > umax_L)
	{
		uk_L = umax_L;
	}
	else if (uk_L < umin_L)
	{
		uk_L = umin_L;
	}
	return(uk_L);
}

float PID_distance(float setpoint, float measure)
{
	ek_2_dis = ek_1_dis;
	ek_1_dis = ek_dis;
	ek_dis = setpoint - measure;
	uk_1_dis = uk_dis;
	uk_dis = uk_1_dis + Kp_dis*(ek_dis-ek_1_dis) + Ki_dis*((sample_ultrasonic_time_ms)/2000)*(ek_dis+ek_1_dis) + (Kd_dis/((sample_ultrasonic_time_ms)/1000))*(ek_dis-2*ek_1_dis+ek_2_dis);
	if(uk_dis > umax_dis)
	{
		uk_dis = umax_dis;
	}
	else if (uk_dis < umin_dis)
	{
		uk_dis = umin_dis;
	}
	return(uk_dis);
}

void PIDtoOmega(float pid_dis)
{
	omega = pid_dis *((PI/2)/400);
	omegaR_SP = (1/r)*v_setpoint + (b/r)*omega;
	omegaL_SP = (1/r)*v_setpoint - (b/r)*omega;
}

