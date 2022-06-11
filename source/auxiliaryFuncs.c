#include "auxiliaryFuncs.h"
#include "dwt_stm32_delay.h"


float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	if (x >= in_max)		x = in_max;
	else if (x <= in_min)   x = in_min;

	return (float) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


float trimAngle(float val, float upper_trim, float lower_trim)
{
	if ((val < upper_trim) && (val > lower_trim)){
		val = 0.0;
	}
	return val;
}

float PWMReadingToRollReferenceAngle(float dutyCycle)
{
	return map(dutyCycle, 1000.0f, 2000.0f, -20.0f, 20.0f);
}


float PWMReadingToPitchReferenceAngle(float dutyCycle)
{
	return map(dutyCycle, 1000.0f, 2000.0f, 20.0f, -20.0f);
}


float PWMReadingToYawRateEror(float yawRate_dutyCycle)
{
	return map(yawRate_dutyCycle, 1000.0f, 2000.0f, -200.0f, 200.0f);
}


float PWMReadingToYawError(float yawDutyCycle)
{
	return map(yawDutyCycle, 1000.0f, 2000.0f, 35.0f, -35.0f);
}


int32_t roundRoll(float val)
{
     int32_t returnRollVal = 0;
	if (val > 0.0){
	    if ((val - 0.5) > (int32_t)val)
	    	returnRollVal = (int32_t)val + 1;
    	else if ((val - 0.5) < (int32_t)val)
    		returnRollVal = (int32_t)val;
	}
	else if (val < 0.0){
		if ((val + 0.5) < (int32_t)val)
			returnRollVal = (int32_t)val - 1;
		else if ((val + 0.5) > (int32_t)val)
			returnRollVal = (int32_t)val;
	}
	return returnRollVal;
}


int32_t roundPitch(float val)
{
	 int32_t returnPitchVal = 0;
	if (val > 0.0){
	    if ((val - 0.5) > (int32_t)val)
	    	returnPitchVal = (int32_t)val + 1;
    	else if ((val - 0.5) < (int32_t)val)
    		returnPitchVal = (int32_t)val;
	}
	else if (val < 0.0){
		if ((val + 0.5) < (int32_t)val)
			returnPitchVal = (int32_t)val - 1;
		else if ((val + 0.5) > (int32_t)val)
			returnPitchVal = (int32_t)val;
	}
	return returnPitchVal;
}


int32_t roundYawError(float val)
{
    int32_t returnYawError = 0;
	if (val > 0.0){
		if ((val - 0.5) > (int32_t)val)
			returnYawError = (int32_t)val + 1;
		else if ((val - 0.5) < (int32_t)val)
			returnYawError = (int32_t)val;
	}
	else if (val < 0.0){
		if ((val + 0.5) < (int32_t)val)
			returnYawError = (int32_t)val - 1;
		else if ((val + 0.5) > (int32_t)val)
			returnYawError = (int32_t)val;
	}
	return returnYawError;
}


int32_t roundThrottle(float throttle)
{
	static int32_t temp;
	static int32_t returnVal;
	temp = throttle - 1000;
	if ((temp % 100) != 0)
		temp = (temp % 100) % 10;
	else if ((temp % 100) == 0)
		temp = temp  % 10;

	if (temp <= 5)
		returnVal = (int32_t)throttle - temp;
	else if (temp > 5)
		returnVal = (int32_t)throttle + (10 - temp);
	return returnVal;
}

float constrain_val(float val, float max, float min)
{
	if (val > max){
		val = max;
	}
	else if (val < min){
		val = min;
	}
	return val;
}

void saturationBlock(uint32_t lf, uint32_t rf, uint32_t lb, uint32_t rb)
{
	if (lf > saturationMax)
		lf = saturationMax;
	else if (lf < saturationMin)
		lf = saturationMin;

	if (rf > saturationMax)
		rf = saturationMax;
	else if (rf < saturationMin)
		rf = saturationMin;

	if (lb > saturationMax)
		lb = saturationMax;
	else if (lb < saturationMin)
		lb = saturationMin;

	if (rb > saturationMax)
		rb = saturationMax;
	else if (rb < saturationMin)
		rb = saturationMin;
}

void motorMixer(float throttle, float roll_out, float pitch_out, float yaw_out)
{
	leftFrontMotor  = (uint32_t)(throttle + roll_out + pitch_out - yaw_out);
	rightFrontMotor = (uint32_t)(throttle - roll_out + pitch_out + yaw_out);
	leftBackMotor   = (uint32_t)(throttle + roll_out - pitch_out + yaw_out);
	rightBackMotor  = (uint32_t)(throttle - roll_out - pitch_out - yaw_out);

	saturationBlock(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);	// limit motors
}

void turn_off_motors(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1000);		// update motor PWMs
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 1000);
}

void motors_output(uint32_t lf, uint32_t rf, uint32_t lb, uint32_t rb)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, rb);    // right front motor
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, lb);    // left back motor
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, rf);    // left front motor
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, lf);    // right back motor
}

void ef_to_bf_conversion(float* rollRate_ref_bf, float* pitchRate_ref_bf, float* yawRate_ref_bf, float rollRate_ef, float pitchRate_ef, float yawRate_ef)
{
	// convert Earth frame angular rates to Body frame sensor rates
	*rollRate_ref_bf  = rollRate_ef - sin(pitchAngle * deg2rad) * yawRate_ef;
	*pitchRate_ref_bf = cos(rollAngle * deg2rad) * pitchRate_ef + cos(pitchAngle * deg2rad) * sin(rollAngle * deg2rad) * yawRate_ef;
	*yawRate_ref_bf   = -sin(pitchAngle * deg2rad) * pitchRate_ef + cos(pitchAngle * deg2rad) * cos(rollAngle * deg2rad) * yawRate_ef;
}

void buzzerOn(void)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	DWT_Delay_us(100000);
}

void buzzerOff(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void activate_buzzer(float temp, float thresh_volt)
{
	if (temp <= thresh_volt)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
