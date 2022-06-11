#include "pid.h"

void pid_outer()
{
	// roll angle pid
	float rollAngle_error = rollAngle_ref - rollAngle;
	float P_val_roll = P_gain_roll * rollAngle_error;

	I_val_roll += rollAngle_error * dt;
	I_val_roll = I_gain_roll * constrain_val(I_val_roll, PID_I_max, PID_I_min);

	float D_val_roll = (rollAngle_error - rollAngle_error_prev) / dt;
	rollAngle_error_prev = rollAngle_error;

	D_val_roll = D_val_roll_prev + (D_val_roll - D_val_roll_prev) * D_FILTER_COFF;
	D_val_roll_prev = D_val_roll;

	PID_val_roll = P_val_roll + I_val_roll + D_gain_roll * D_val_roll;


	//pitch angle pid
	float pitchAngle_error = pitchAngle_ref - pitchAngle;
	float P_val_pitch = P_gain_pitch * pitchAngle_error;

	I_val_pitch += pitchAngle_error * dt;
	I_val_pitch = I_gain_pitch * constrain_val(I_val_pitch, PID_I_max, PID_I_min);

	float D_val_pitch = (pitchAngle_error - pitchAngle_error_prev) / dt;
	pitchAngle_error_prev = pitchAngle_error;
	D_val_pitch = D_val_pitch + (D_val_pitch - D_val_pitch_prev) * D_FILTER_COFF;
	D_val_pitch_prev = D_val_pitch;

	PID_val_pitch = P_val_pitch + I_val_pitch + D_gain_pitch * D_val_pitch;
}


void pid_inner(float _rollRate_ref_bf, float _pitchRate_ref_bf, float _yawRate_ref_bf, float* gyro)
{
	//roll rate pid
	float _rollRate_error = _rollRate_ref_bf - gyro[0];

	float P_val_roll_angularRate = P_gain_roll_angularRate * _rollRate_error;
	I_val_roll_angularRate += I_gain_roll_angularRate *  _rollRate_error * dt_ar;
	I_val_roll_angularRate =  constrain_val(I_val_roll_angularRate, PID_I_val_angularRate_max, PID_I_val_angularRate_min);

	float D_val_roll_angularRate = (_rollRate_error - _rollRate_error_prev) / dt_ar;
	_rollRate_error_prev = _rollRate_error;
	D_val_roll_angularRate = D_val_roll_angularRate_prev + (D_val_roll_angularRate - D_val_roll_angularRate_prev) * D_FILTER_COFF;
	D_val_roll_angularRate_prev = D_val_roll_angularRate;

	PID_val_roll_angularRate = P_val_roll_angularRate + I_val_roll_angularRate + D_gain_roll_angularRate * D_val_roll_angularRate;
	PID_val_roll_angularRate = constrain_val(PID_val_roll_angularRate, PID_val_angularRate_max, PID_val_angularRate_min);

	//pitch rate pid
	float _pitchRate_error = _pitchRate_ref_bf - gyro[1];

	float P_val_pitch_angularRate = P_gain_pitch_angularRate * _pitchRate_error;
	I_val_pitch_angularRate += I_gain_pitch_angularRate * _pitchRate_error * dt_ar;
	I_val_pitch_angularRate =  constrain_val(I_val_pitch_angularRate, PID_I_val_angularRate_max, PID_I_val_angularRate_min);

	float D_val_pitch_angularRate = (_pitchRate_error - _pitchRate_error_prev) / dt_ar;
	_pitchRate_error_prev = _pitchRate_error;
	D_val_pitch_angularRate = D_val_pitch_angularRate_prev + (D_val_pitch_angularRate - D_val_pitch_angularRate_prev) * D_FILTER_COFF;
	D_val_pitch_angularRate_prev = D_val_pitch_angularRate;

	PID_val_pitch_angularRate = P_val_pitch_angularRate + I_val_pitch_angularRate + D_gain_pitch_angularRate * D_val_pitch_angularRate;
	PID_val_pitch_angularRate = constrain_val(PID_val_pitch_angularRate, PID_val_angularRate_max, PID_val_angularRate_min);

	// yaw rate pid
	float _yawRate_error = _yawRate_ref_bf - gyro[2];

	float P_val_yaw_angularRate = P_gain_yaw_angularRate * _yawRate_error;
	I_val_yaw_angularRate += I_gain_yaw_angularRate *  _yawRate_error * dt_ar;
	I_val_yaw_angularRate = I_gain_yaw_angularRate * constrain_val(I_val_yaw_angularRate, PID_I_val_angularRate_max, PID_I_val_angularRate_min);

	float D_val_yaw_angularRate = (_yawRate_error - _yawRate_error_prev) / dt_ar;
	_yawRate_error_prev = _yawRate_error;
	D_val_yaw_angularRate = D_val_yaw_angularRate_prev + (D_val_yaw_angularRate - D_val_yaw_angularRate_prev) * D_FILTER_COFF;
	D_val_yaw_angularRate_prev = D_val_yaw_angularRate;

	PID_val_yaw_angularRate = P_val_yaw_angularRate + I_val_yaw_angularRate + D_gain_yaw_angularRate * D_val_yaw_angularRate;
	PID_val_yaw_angularRate = constrain_val(PID_val_yaw_angularRate, PID_val_angularRate_max, PID_val_angularRate_min);
}

void resetPID(void)
{
	PID_val_roll  = 0.0f;
	PID_val_pitch = 0.0f;

    I_val_roll = 0.0f;
	I_val_pitch = 0.0f;
	I_val_yaw = 0.0f;

	D_val_pitch_prev = 0.0f;
	D_val_roll_prev = 0.0f;
	D_val_yaw_prev = 0.0f;
}
void resetPID_ar(void)
{
	PID_val_roll_angularRate   = 0.0f;
	PID_val_pitch_angularRate  = 0.0f;
	PID_val_yaw_angularRate    = 0.0f;

	I_val_roll_angularRate  = 0.0f;
	I_val_pitch_angularRate = 0.0f;
	I_val_yaw_angularRate   = 0.0f;

	D_val_roll_angularRate_prev  = 0.0f;
	D_val_pitch_angularRate_prev = 0.0f;
	D_val_yaw_angularRate_prev   = 0.f;
}
