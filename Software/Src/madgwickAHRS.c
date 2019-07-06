#include "madgwickAHRS.h"
#include <math.h>

void madgwickInit(madgwickAhrs_t *sensor, float beta, float freq)
{
    sensor->beta = beta;
	sensor->q0 = 1.0f;
	sensor->q1 = 0.0f;
	sensor->q2 = 0.0f;
	sensor->q3 = 0.0f;
	sensor->invSampleFreq = 1.0f / freq;
	sensor->anglesComputed = 0;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void computeAngles(madgwickAhrs_t *sensor)
{
	sensor->roll = atan2f(sensor->q0*sensor->q1 + sensor->q2*sensor->q3, 0.5f - sensor->q1*sensor->q1 - sensor->q2*sensor->q2);
	sensor->pitch = asinf(-2.0f * (sensor->q1*sensor->q3 - sensor->q0*sensor->q2));
	sensor->yaw = atan2f(sensor->q1*sensor->q2 + sensor->q0*sensor->q3, 0.5f - sensor->q2*sensor->q2 - sensor->q3*sensor->q3);
	sensor->anglesComputed = 1;
}

float getRoll(madgwickAhrs_t *sensor) {
    if (!sensor->anglesComputed) computeAngles(sensor);
    return sensor->roll * 57.29578f;
}

float getPitch(madgwickAhrs_t *sensor) {
    if (!sensor->anglesComputed) computeAngles(sensor);
    return sensor->pitch * 57.29578f;
}

float getYaw(madgwickAhrs_t *sensor) {
    if (!sensor->anglesComputed) computeAngles(sensor);
    return sensor->yaw * 57.29578f + 180.0f;
}

float getRollRadians(madgwickAhrs_t *sensor) {
    if (!sensor->anglesComputed) computeAngles(sensor);
    return sensor->roll;
}

float getPitchRadians(madgwickAhrs_t *sensor) {
    if (!sensor->anglesComputed) computeAngles(sensor);
    return sensor->pitch;
}

float getYawRadians(madgwickAhrs_t *sensor) {
    if (!sensor->anglesComputed) computeAngles(sensor);
    return sensor->yaw;
}

void madgwickUpdateIMU(madgwickAhrs_t *sensor, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-sensor->q1 * gx - sensor->q2 * gy - sensor->q3 * gz);
	qDot2 = 0.5f * (sensor->q0 * gx + sensor->q2 * gz - sensor->q3 * gy);
	qDot3 = 0.5f * (sensor->q0 * gy - sensor->q1 * gz + sensor->q3 * gx);
	qDot4 = 0.5f * (sensor->q0 * gz + sensor->q1 * gy - sensor->q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * sensor->q0;
		_2q1 = 2.0f * sensor->q1;
		_2q2 = 2.0f * sensor->q2;
		_2q3 = 2.0f * sensor->q3;
		_4q0 = 4.0f * sensor->q0;
		_4q1 = 4.0f * sensor->q1;
		_4q2 = 4.0f * sensor->q2;
		_8q1 = 8.0f * sensor->q1;
		_8q2 = 8.0f * sensor->q2;
		q0q0 = sensor->q0 * sensor->q0;
		q1q1 = sensor->q1 * sensor->q1;
		q2q2 = sensor->q2 * sensor->q2;
		q3q3 = sensor->q3 * sensor->q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * sensor->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * sensor->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * sensor->q3 - _2q1 * ax + 4.0f * q2q2 * sensor->q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= sensor->beta * s0;
		qDot2 -= sensor->beta * s1;
		qDot3 -= sensor->beta * s2;
		qDot4 -= sensor->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	sensor->q0 += qDot1 * sensor->invSampleFreq;
	sensor->q1 += qDot2 * sensor->invSampleFreq;
	sensor->q2 += qDot3 * sensor->invSampleFreq;
	sensor->q3 += qDot4 * sensor->invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(sensor->q0 * sensor->q0 + sensor->q1 * sensor->q1 + sensor->q2 * sensor->q2 + sensor->q3 * sensor->q3);
	sensor->q0 *= recipNorm;
	sensor->q1 *= recipNorm;
	sensor->q2 *= recipNorm;
	sensor->q3 *= recipNorm;
	sensor->anglesComputed = 0;
}

void madgwickUpdate(madgwickAhrs_t *sensor, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		madgwickUpdateIMU(sensor, gx, gy, gz, ax, ay, az);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-sensor->q1 * gx - sensor->q2 * gy - sensor->q3 * gz);
	qDot2 = 0.5f * (sensor->q0 * gx + sensor->q2 * gz - sensor->q3 * gy);
	qDot3 = 0.5f * (sensor->q0 * gy - sensor->q1 * gz + sensor->q3 * gx);
	qDot4 = 0.5f * (sensor->q0 * gz + sensor->q1 * gy - sensor->q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * sensor->q0 * mx;
		_2q0my = 2.0f * sensor->q0 * my;
		_2q0mz = 2.0f * sensor->q0 * mz;
		_2q1mx = 2.0f * sensor->q1 * mx;
		_2q0 = 2.0f * sensor->q0;
		_2q1 = 2.0f * sensor->q1;
		_2q2 = 2.0f * sensor->q2;
		_2q3 = 2.0f * sensor->q3;
		_2q0q2 = 2.0f * sensor->q0 * sensor->q2;
		_2q2q3 = 2.0f * sensor->q2 * sensor->q3;
		q0q0 = sensor->q0 * sensor->q0;
		q0q1 = sensor->q0 * sensor->q1;
		q0q2 = sensor->q0 * sensor->q2;
		q0q3 = sensor->q0 * sensor->q3;
		q1q1 = sensor->q1 * sensor->q1;
		q1q2 = sensor->q1 * sensor->q2;
		q1q3 = sensor->q1 * sensor->q3;
		q2q2 = sensor->q2 * sensor->q2;
		q2q3 = sensor->q2 * sensor->q3;
		q3q3 = sensor->q3 * sensor->q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * sensor->q3 + _2q0mz * sensor->q2 + mx * q1q1 + _2q1 * my * sensor->q2 + _2q1 * mz * sensor->q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * sensor->q3 + my * q0q0 - _2q0mz * sensor->q1 + _2q1mx * sensor->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * sensor->q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * sensor->q2 + _2q0my * sensor->q1 + mz * q0q0 + _2q1mx * sensor->q3 - mz * q1q1 + _2q2 * my * sensor->q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * sensor->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * sensor->q3 + _2bz * sensor->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * sensor->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * sensor->q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * sensor->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * sensor->q2 + _2bz * sensor->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * sensor->q3 - _4bz * sensor->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * sensor->q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * sensor->q2 - _2bz * sensor->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * sensor->q1 + _2bz * sensor->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * sensor->q0 - _4bz * sensor->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * sensor->q3 + _2bz * sensor->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * sensor->q0 + _2bz * sensor->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * sensor->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= sensor->beta * s0;
		qDot2 -= sensor->beta * s1;
		qDot3 -= sensor->beta * s2;
		qDot4 -= sensor->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	sensor->q0 += qDot1 * sensor->invSampleFreq;
	sensor->q1 += qDot2 * sensor->invSampleFreq;
	sensor->q2 += qDot3 * sensor->invSampleFreq;
	sensor->q3 += qDot4 * sensor->invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(sensor->q0 * sensor->q0 + sensor->q1 * sensor->q1 + sensor->q2 * sensor->q2 + sensor->q3 * sensor->q3);
	sensor->q0 *= recipNorm;
	sensor->q1 *= recipNorm;
	sensor->q2 *= recipNorm;
	sensor->q3 *= recipNorm;
	sensor->anglesComputed = 0;
}