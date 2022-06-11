/*
 * auxiliaryFuncs.h
 *
 *  Created on: Jul 16, 2018
 *      Author: Nijat Ibrahimov
 */


#ifndef AUXILIARYFUNCS_H_
#define AUXILIARYFUNCS_H_

#include <stdint.h>
#include <math.h>
#include "stm32f4xx_hal.h"

#define Pi              3.14159265f
#define deg2rad 		Pi / 180
#define rad2deg			180 / Pi

#define   saturationMax					1900
#define   saturationMin					1000

extern TIM_HandleTypeDef htim4;

extern volatile float rollAngle, pitchAngle;
extern uint32_t leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;


float map(float x, float in_min, float in_max, float out_min, float out_max);
float PWMReadingToRollReferenceAngle(float dutyCycle);
float PWMReadingToPitchReferenceAngle(float dutyCycle);
float PWMReadingToYawError(float yawDutyCycle);
float PWMReadingToYawRateEror(float yawRate_dutyCycle);
int32_t roundRoll(float val);
int32_t roundPitch(float val);
int32_t roundYawError(float val);
int32_t roundThrottle(float throttle);
float constrain_val(float val, float max, float min);

void buzzerOn(void);
void buzzerOff(void);
void activate_buzzer(float temp, float thresh_volt);

void saturationBlock(uint32_t lf, uint32_t rf, uint32_t lb, uint32_t rb);
void motorMixer(float throttle, float roll_out, float pitch_out, float yaw_out);
void turn_off_motors(void);
void motors_output(uint32_t lf, uint32_t rf, uint32_t lb, uint32_t rb);
void ef_to_bf_conversion(float* rollRate_ref_bf, float* pitchRate_ref_bf, float* yawRate_ref_bf, float rollRate_ef, float pitchRate_ef, float yawRate_ef);
float trimAngle(float angle, float upper_trim, float lower_trim);


#endif /* AUXILIARYFUNCS_H_ */
