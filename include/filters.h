/*
 * filters.h
 *
 *  Created on: Jul 16, 2018
 *      Author: Nijat Ibrahimov
 */

#ifndef FILTERS_H_
#define FILTERS_H_

#include "stm32f4xx_hal.h"
#include "auxiliaryFuncs.h"
#include <math.h>
#include <stdbool.h>

#define   rollAngle_max 	80.0f
#define   rollAngle_min		-80.0f
#define   pitchAngle_max	80.0f
#define   pitchAngle_min	-80.0f

#define   reverseRoll		1 	//true
#define   reversePitch      0 	//false

#define Pi              3.14159265f
#define Dt              0.004f			// complementary filter sampling period
#define alpha     		0.98f			// Complementary filter coefficient

#define Kp 				2.0f * 0.6f	// Mahony filter PI parameters
#define Ki 				2.0f * 0.008f

#define sampleFreq 		1000 			// Frequency in Hz
#define betaDef  		0.1  			// Madgwick filter coefficient

#define deg2rad 		Pi / 180
#define rad2deg			180 / Pi


extern float rollAngle_prev, pitchAngle_prev;
extern volatile float rollAngle, pitchAngle;

extern float prev_gyroData[3], prev_accelData[3], prev_gyroData2[3], prev_AccelData2[3];

extern volatile float q0, q1, q2, q3;
extern volatile float integralFBx, integralFBy, integralFBz;	//integral error terms scaled by Ki

volatile float twoKp;
volatile float twoKi;
volatile float beta;


void complementaryFilter_v1(float* accelData, float* gyroData, float* roll, float* pitch);
void complementaryFilter_v2(float* accelData, float* gyroData, float* roll, float* pitch);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void calculateAnglesFromQuaternion(void);

void ahrs_fusion_ag(float* acc, float* gyro);
//static float invSqrt(float x);

void lowPassGyro(float* gyroData);
void lowPassAccel(float* accelData);
void lowPassGyro_MoveAve(float* gyroData);
void lowPassGyro_MoveAve_new(float* gyroData, float* gyroData_filt);

#endif /* FILTERS_H_ */
