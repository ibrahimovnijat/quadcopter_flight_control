#include "filters.h"


volatile float twoKp = Kp;
volatile float twoKi = Ki;
volatile float beta = betaDef;

static float invSqrt(float x);


void complementaryFilter(float* accelData, float* gyroData, float* roll, float* pitch)
{
	float accRoll  = atan2((float)accelData[1], (float)sqrt(accelData[0]*accelData[0] + accelData[2]*accelData[2])) * rad2deg;
	float accPitch = atan2((float)accelData[0], (float)sqrt(accelData[1]*accelData[1] + accelData[2]*accelData[2])) * (-rad2deg);

	*roll = rollAngle_prev + gyroData[0] * Dt;
	float rollAngle_error = accRoll - *roll;
	*roll = rollAngle_prev + (1 - alpha) * rollAngle_error;
	rollAngle_prev = *roll;

	*pitch = pitchAngle_prev + gyroData[1] * Dt;
	float pitchAngle_error = accPitch - *pitch;
	*pitch = pitchAngle_prev + (1 - alpha) * pitchAngle_error;
	pitchAngle_prev = *pitch;
}


void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		// Normalize accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f;	// prevent integral wind-up
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalize quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	//Convert gyro readings from degree to radians
	gx = gx * deg2rad;
	gy = gy * deg2rad;
	gz = gz * deg2rad;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalize accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalize step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalize quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void calculateAnglesFromQuaternion()
{
	rollAngle  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * rad2deg;
	pitchAngle = -asin(2.0f * (q1 * q3 - q0 * q2)) * rad2deg;

	if(reverseRoll)
		rollAngle = -rollAngle;

	if (reversePitch)
		pitchAngle = -pitchAngle;

	rollAngle  = constrain_val(rollAngle, rollAngle_max, rollAngle_min);
	pitchAngle = constrain_val(pitchAngle, pitchAngle_max, pitchAngle_min);
}

static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void lowPassGyro(float* gyroData)
{
	const float lowPassCoeff = 0.7f;
	gyroData[0] = lowPassCoeff * gyroData[0] + (1 - lowPassCoeff) * prev_gyroData[0];
	prev_gyroData[0] = gyroData[0];

	gyroData[1] = lowPassCoeff * gyroData[1] + (1 - lowPassCoeff) * prev_gyroData[1];
	prev_gyroData[1] = gyroData[1];

	gyroData[2] = lowPassCoeff * gyroData[2] + (1 - lowPassCoeff) * prev_gyroData[2];
	prev_gyroData[2] = gyroData[2];
}



void lowPassGyro_MoveAve(float* gyroData, float* gyroData_filt)
{
	gyroData_filt[0] = (gyroData[0] + prev_gyroData[0] + prev_gyroData2[0]) / 3.0f;
	prev_gyroData2[0] = prev_gyroData[0];
	prev_gyroData[0] = gyroData[0];

	gyroData_filt[1] = (gyroData[1] + prev_gyroData[1] + prev_gyroData2[1]) / 3.0f;
	prev_gyroData2[1] = prev_gyroData[1];
	prev_gyroData[1] = gyroData[1];

	gyroData_filt[2] = (gyroData[2] + prev_gyroData[2] + prev_gyroData2[2]) / 3.0f;
	prev_gyroData2[2] = prev_gyroData[2];
	prev_gyroData[2] = gyroData[2];
}

void lowPassAccel(float* accelData)
{
	const float lowPassCoeff = 0.7f;
	accelData[0] = lowPassCoeff * accelData[0] + (1 - lowPassCoeff) * prev_accelData[0];
	prev_accelData[0] = accelData[0];

	accelData[1] = lowPassCoeff * accelData[1] + (1 - lowPassCoeff) * prev_accelData[1];
	prev_accelData[1] = accelData[1];

	accelData[2] = lowPassCoeff * accelData[2] + (1 - lowPassCoeff) * prev_accelData[2];
	prev_accelData[2] = accelData[2];
}
