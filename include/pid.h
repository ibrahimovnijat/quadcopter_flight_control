/*
 * st_pid.h
 *
 *  Created on: July 23, 2018
 *      Author: Nijat Ibrahaimov
 */

#ifndef ST_PID_H_
#define ST_PID_H_

#include "auxiliaryFuncs.h"

#define   dt 							0.01f 		// 100Hz, 10 ms period;  outer loop PID freq (angle PID)
#define   dt_ar							0.0025f	    // 400Hz, 2.5 ms period; inner loop PID freq (angular rate PID)

#define   PID_max						300.0f
#define   PID_min						-300.0f
#define   PID_I_max						100.0f
#define   PID_I_min						-100.0f
#define   PID_val_angularRate_max		200.0f
#define   PID_val_angularRate_min		-200.0f
#define   PID_I_val_angularRate_max		100.0f
#define   PID_I_val_angularRate_min		-100.0f

#define   D_FILTER_COFF       			0.025f


extern const float P_gain_roll, I_gain_roll, D_gain_roll;
extern const float P_gain_pitch, I_gain_pitch, D_gain_pitch;
extern const float P_gain_yaw, I_gain_yaw, D_gain_yaw;

extern volatile float rollAngle, pitchAngle;
extern float rollAngle_prev, pitchAngle_prev;
extern volatile float rollAngle_ref, pitchAngle_ref, yawAngle_error;
extern float rollAngle_ref_prev, pitchAngle_ref_prev, yawAngle_error_prev;

extern volatile float rollRate_ref, pitchRate_ref, yawRate_ref;
extern float rollRate_ref_prev, pitchRate_ref_prev, yawRate_ref_prev;

extern float I_val_roll, I_val_pitch, I_val_yaw;
extern float D_val_roll, D_val_pitch, D_val_yaw, D_val_roll_prev, D_val_pitch_prev, D_val_yaw_prev;
extern volatile float PID_val_roll, PID_val_pitch; // PID_val_yaw;

/*Variable for angular rate PID function --------------*/
extern const float  P_gain_roll_angularRate,  I_gain_roll_angularRate,  D_gain_roll_angularRate;
extern const float	 P_gain_pitch_angularRate, I_gain_pitch_angularRate, D_gain_pitch_angularRate;
extern const float	 P_gain_yaw_angularRate,   I_gain_yaw_angularRate,   D_gain_yaw_angularRate;

extern float I_val_roll_angularRate, I_val_pitch_angularRate, I_val_yaw_angularRate;
extern float rollPid_prev, pitchPid_prev, yawPid_prev;
extern volatile float PID_val_roll_angularRate, PID_val_pitch_angularRate, PID_val_yaw_angularRate;
extern float gyroscopeData_roll_prev, gyroscopeData_pitch_prev, gyroscopeData_yaw_prev;
extern float D_val_roll_angularRate_prev, D_val_pitch_angularRate_prev, D_val_yaw_angularRate_prev;

/*----- new variables for ST PID ----------*/
extern float _rollRate_error_prev, _pitchRate_error_prev, _yawRate_error_prev;
extern float rollAngle_error_prev, pitchAngle_error_prev;

/*----------------------- Functions for st_pid ---------------------*/
void pid_outer();
void pid_inner(float _rollRate_ref_bf, float _pitchRate_ref_bf, float _yawRate_ref_bf, float* gyro);

void resetPID(void);
void resetPID_ar(void);

#endif /* ST_PID_H_ */
