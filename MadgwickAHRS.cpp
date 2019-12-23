//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
// 19/08/2014			Patched - https://diydrones.com/xn/detail/705844:Comment:1755084
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

MadgwickAHRS::MadgwickAHRS(double beta) : beta(beta) {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    a0 = 0.0;
    a1 = 0.0;
    a2 = 0.0;

    v0 = 0.0;
    v1 = 0.0;
    v2 = 0.1;

    x = 0;
    y = 0;
    z = 0;

    aSigma = 0;
}

void MadgwickAHRS::update(double dt, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz) {
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    double _4bx, _4bz, _8bx, _8bz, _2q0, _2q1, _2q2, _2q3;
    double _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)

    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
	updateIMU(dt, gx, gy, gz, ax, ay, az);
	return;
    }

    // Convert gyroscope degrees/sec to radians/sec

    gx *= 0.0174533;
    gy *= 0.0174533;
    gz *= 0.0174533;

    // Rate of change of quaternion from gyroscope

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
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

	_2q0mx = 2.0f * q0 * mx;
	_2q0my = 2.0f * q0 * my;
	_2q0mz = 2.0f * q0 * mz;
	_2q1mx = 2.0f * q1 * mx;
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q0q2 = 2.0f * q0 * q2;
	_2q2q3 = 2.0f * q2 * q3;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Reference direction of Earth's magnetic field

	hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	_8bx = 2.0f * _4bx;
	_8bz = 2.0f * _4bz;

	// Gradient decent algorithm corrective step

	s0= -_2q2*(2.0f*(q1q3 - q0q2) - ax) + _2q1*(2.0f*(q0q1 + q2q3) - ay) + -_4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx) + (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
	s1= _2q3*(2.0f*(q1q3 - q0q2) - ax) + _2q0*(2.0f*(q0q1 + q2q3) - ay) + -4.0f*q1*(2.0f*(0.5 - q1q1 - q2q2) - az) + _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);             
	s2= -_2q0*(2.0f*(q1q3 - q0q2) - ax) + _2q3*(2.0f*(q0q1 + q2q3) - ay) + (-4.0f*q2)*(2.0f*(0.5 - q1q1 - q2q2) - az) + (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
	s3= _2q1*(2.0f*(q1q3 - q0q2) - ax) + _2q2*(2.0f*(q0q1 + q2q3) - ay)+(-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);

	recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void MadgwickAHRS::updateIMU(double dt, double gx, double gy, double gz, double ax, double ay, double az) {
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec

    gx *= 0.0174533;
    gy *= 0.0174533;
    gz *= 0.0174533;

    // Rate of change of quaternion from gyroscope

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
	// Normalise accelerometer measurement

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

	recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

double MadgwickAHRS::invSqrt(double x) {
    return 1.0/sqrt(x);
}

void MadgwickAHRS::getAngles(double *roll, double *pitch, double *yaw) {
    double a12 = 2.0f * (q1 * q2 + q0 * q3);
    double a22 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    double a31 = 2.0f * (q0 * q1 + q2 * q3);
    double a32 = 2.0f * (q1 * q3 - q0 * q2);
    double a33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    *roll = atan2f(a31, a33);
    *pitch = -asinf(a32);
    *yaw = atan2f(a12, a22);
}

void MadgwickAHRS::gravityCompensate(double ax, double ay, double az) {
    double g0 = 2.0 * (q1 * q3 - q0 * q2);
    double g1 = 2.0 * (q0 * q1 + q2 * q3);
    double g2 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    a0 = (ax - g0);
    a1 = (ay - g1);
    a2 = (az - g2);
}

void MadgwickAHRS::integrate(double dt) {
    double r = sqrt(a0 * a0 + a1 * a1 + a2 * a2);

    if (r > aSigma) {
	v0 += a0 * dt * 9.80665;
	v1 += a1 * dt * 9.80665;
	v2 += a2 * dt * 9.80665;
    } else {
	v0 *= 0.75;
	v1 *= 0.75;
	v2 *= 0.75;
    }

    x += v0 * dt;
    y += v1 * dt;
    z += v2 * dt;
}

void MadgwickAHRS::setAccelSigma(double x) {
    aSigma = x;
}
