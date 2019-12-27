#include <inttypes.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <iostream>

#include <Eigen/Dense>

#include "Compensation.h"
#include "Control.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"

#define PI 3.141526

using namespace cacaosd_i2cport;
using namespace cacaosd_mpu6050;
using namespace cacaosd_hmc5883l;
using namespace wampcc;
using namespace Eigen;

MadgwickAHRS	imu(0.3);
Compensation	comp;
Control		control(&comp, &imu);

MPU6050		*mpu6050 = NULL;
HMC5883L	*hmc5883L = NULL;

Vector3i	accel;
Vector3i	gyro;
Vector3i	mag;

Vector3d	accelCal;
Vector3d	gyroCal;
Vector3d	magCal;

double		dt = 1.0/500.0;

double time_ns() {
    struct timespec spec;

    clock_gettime(CLOCK_MONOTONIC, &spec);

    return spec.tv_sec + spec.tv_nsec / 1.0e9;
}

void mag_work(union sigval) {
    hmc5883L->getMagnitude(mag);
    comp.doMag(mag, magCal);
}

void gyro_work(union sigval) {
    mpu6050->getAcceleration(accel);
    mpu6050->getAngularVelocity(gyro);

    comp.doAccel(accel, accelCal);
    comp.doGyro(gyro, gyroCal);

    accelCal *= 2.0 / 32768.0;
    gyroCal *= 1000.0 / 32768;
    gyroCal *= 0.0174533;

    if (hmc5883L) {
	imu.update(dt, gyroCal, accelCal, magCal);
    } else {
	imu.update(dt, gyroCal, accelCal);
    }

    imu.gravityCompensate(accelCal);
    imu.integrate(dt);
}

void gyro_timer() {
    struct itimerspec	its;
    timer_t		timerid;
    struct sigevent	sev;

    memset((void *) &sev, 0, sizeof(sev));

    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = &gyro_work;

    timer_create(CLOCK_REALTIME, &sev, &timerid);

    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 1000000000 * dt;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;

    timer_settime(timerid, 0, &its, NULL);
}

void mag_timer() {
    struct itimerspec	its;
    timer_t		timerid;
    struct sigevent	sev;

    memset((void *) &sev, 0, sizeof(sev));

    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = &mag_work;

    timer_create(CLOCK_REALTIME, &sev, &timerid);

    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 1000000000 / 75;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;

    timer_settime(timerid, 0, &its, NULL);
}

int main() {
    control.loadConfig("imu.json");
    control.init();

    I2cPort *i2c0 = new I2cPort(0x68, 0);
    i2c0->openConnection();

    if (i2c0->isConnectionOpen()) {
	mpu6050 = new MPU6050(i2c0);

        mpu6050->setRangeAcceleration(0);	// 2g
        mpu6050->setRangeGyroscope(2);		// 1000 gr/s
	mpu6050->setSampleRate(0);
        mpu6050->setDLPFMode(6);
        mpu6050->setSleepMode(false);

        control.setGyroscop(&gyro);
        control.setAccelerometer(&accel);
    } else {
        exit(1);
    }

    I2cPort *i2c1 = new I2cPort(0x1E, 0);
    i2c1->openConnection();

    if (i2c1->isConnectionOpen()) {
	hmc5883L = new HMC5883L(i2c1);

        hmc5883L->initialize();
        hmc5883L->setOutputRate(OUTPUT_RATE_6);	// 75Hz

        control.setMagnetometer(&mag);
    } else {
	exit(1);
    }

    gyro_timer();

    if (hmc5883L) {
	mag_timer();
    }

    imu.setAccelSigma(0.002);

    while (true) {
	sleep(1);
    }

    return 0;
}
