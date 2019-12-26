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

    vel = Vector3d::Zero();
    pos = Vector3d::Zero();

    aSigma = 0;
}

void MadgwickAHRS::update(double dt, Vector3d gyro, Vector3d accel, Vector3d mag) {
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    double _4bx, _4bz, _8bx, _8bz, _2q0, _2q1, _2q2, _2q3;
    double _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope

    qDot1 = 0.5f * (-q1 * gyro(0) - q2 * gyro(1) - q3 * gyro(2));
    qDot2 = 0.5f * (q0 * gyro(0) + q2 * gyro(2) - q3 * gyro(1));
    qDot3 = 0.5f * (q0 * gyro(1) - q1 * gyro(2) + q3 * gyro(0));
    qDot4 = 0.5f * (q0 * gyro(2) + q1 * gyro(1) - q2 * gyro(0));

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

    if (!((accel(0) == 0.0f) && (accel(1) == 0.0f) && (accel(2) == 0.0f))) {
	accel.normalize();
	mag.normalize();

	// Auxiliary variables to avoid repeated arithmetic

	_2q0mx = 2.0f * q0 * mag(0);
	_2q0my = 2.0f * q0 * mag(1);
	_2q0mz = 2.0f * q0 * mag(2);
	_2q1mx = 2.0f * q1 * mag(0);
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

	hx = mag(0) * q0q0 - _2q0my * q3 + _2q0mz * q2 + mag(0) * q1q1 + _2q1 * mag(1) * q2 + _2q1 * mag(2) * q3 - mag(0) * q2q2 - mag(0) * q3q3;
	hy = _2q0mx * q3 + mag(1) * q0q0 - _2q0mz * q1 + _2q1mx * q2 - mag(1) * q1q1 + mag(1) * q2q2 + _2q2 * mag(2) * q3 - mag(1) * q3q3;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mag(2) * q0q0 + _2q1mx * q3 - mag(2) * q1q1 + _2q2 * mag(1) * q3 - mag(2) * q2q2 + mag(2) * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	_8bx = 2.0f * _4bx;
	_8bz = 2.0f * _4bz;

	// Gradient decent algorithm corrective step

	s0= -_2q2*(2.0f*(q1q3 - q0q2) - accel(0)) + _2q1*(2.0f*(q0q1 + q2q3) - accel(1)) + -_4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mag(0)) + (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - mag(1))    +   _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mag(2));
	s1= _2q3*(2.0f*(q1q3 - q0q2) - accel(0)) + _2q0*(2.0f*(q0q1 + q2q3) - accel(1)) + -4.0f*q1*(2.0f*(0.5 - q1q1 - q2q2) - accel(2)) + _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mag(0))   + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - mag(1))   +   (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mag(2));
	s2= -_2q0*(2.0f*(q1q3 - q0q2) - accel(0)) + _2q3*(2.0f*(q0q1 + q2q3) - accel(1)) + (-4.0f*q2)*(2.0f*(0.5 - q1q1 - q2q2) - accel(2)) + (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mag(0))+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - mag(1))+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mag(2));
	s3= _2q1*(2.0f*(q1q3 - q0q2) - accel(0)) + _2q2*(2.0f*(q0q1 + q2q3) - accel(1))+(-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mag(0))+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - mag(1))+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mag(2));

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

void MadgwickAHRS::update(double dt, Vector3d gyro, Vector3d accel) {
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q1 * gyro(0) - q2 * gyro(1) - q3 * gyro(2));
    qDot2 = 0.5f * (q0 * gyro(0) + q2 * gyro(2) - q3 * gyro(1));
    qDot3 = 0.5f * (q0 * gyro(1) - q1 * gyro(2) + q3 * gyro(0));
    qDot4 = 0.5f * (q0 * gyro(2) + q1 * gyro(1) - q2 * gyro(0));

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

    if (!((accel(0) == 0.0f) && (accel(1) == 0.0f) && (accel(2) == 0.0f))) {
	accel.normalize();

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

	s0 = _4q0 * q2q2 + _2q2 * accel(0) + _4q0 * q1q1 - _2q1 * accel(1);
	s1 = _4q1 * q3q3 - _2q3 * accel(0) + 4.0f * q0q0 * q1 - _2q0 * accel(1) - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accel(2);
	s2 = 4.0f * q0q0 * q2 + _2q0 * accel(0) + _4q2 * q3q3 - _2q3 * accel(1) - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accel(2);
	s3 = 4.0f * q1q1 * q3 - _2q1 * accel(0) + 4.0f * q2q2 * q3 - _2q2 * accel(1);

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

void MadgwickAHRS::getAngles(Vector3d &res) {
    double a12 = 2.0f * (q1 * q2 + q0 * q3);
    double a22 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    double a31 = 2.0f * (q0 * q1 + q2 * q3);
    double a32 = 2.0f * (q1 * q3 - q0 * q2);
    double a33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    res(0) = atan2(a31, a33);
    res(1) = -asin(a32);
    res(2) = atan2(a12, a22);
}

void MadgwickAHRS::gravityCompensate(Vector3d &accel) {
    Vector3d g;

    g(0) = 2.0 * (q1 * q3 - q0 * q2);
    g(1) = 2.0 * (q0 * q1 + q2 * q3);
    g(2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    this->accel = accel - g;
}

void MadgwickAHRS::integrate(double dt) {
    if (accel.norm() > aSigma) {
	vel += accel * dt * 9.80665;
    } else {
	vel *= 0.75;
    }

    pos = vel * dt;
}

void MadgwickAHRS::setAccelSigma(double x) {
    aSigma = x;
}
