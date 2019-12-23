#include <inttypes.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <iostream>

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

MadgwickAHRS	imu(0.3);
Compensation	comp;
Control		control(&comp, &imu);

MPU6050		*mpu6050 = NULL;
HMC5883L	*hmc5883L = NULL;
double		mx = 0, my = 0, mz = 0;
double		dt = 1.0/500.0;

double time_ns() {
    struct timespec spec;

    clock_gettime(CLOCK_MONOTONIC, &spec);

    return spec.tv_sec + spec.tv_nsec / 1.0e9;
}

void mag_work(union sigval) {
    mx = hmc5883L->getMagnitudeX();
    my = hmc5883L->getMagnitudeY();
    mz = hmc5883L->getMagnitudeZ();
}

void gyro_work(union sigval) {
    int16_t	m[9];
    double	d[9];

    mpu6050->getMotions6(m);

    m[6] = mx;
    m[7] = my;
    m[8] = mz;

    comp.doIt(m, d);

    for (int i = 0; i < 3; i++) {
	d[i] = d[i] * 2.0 / 32768.0;
	d[i+3] = d[i+3] * 1000.0 / 32768.0;
	d[i+6] = d[i+6] / 1024.0;
    }

    if (hmc5883L) {
	imu.update(dt,  d[3], d[4], d[5],  d[0], d[1], d[2],  d[6], d[7], d[8]);
    } else {
	imu.updateIMU(dt,  d[3], d[4], d[5],  d[0], d[1], d[2]);
    }

    imu.gravityCompensate(d[0], d[1], d[2]);
    imu.integrate(dt);
}

void calibrate_work(union sigval) {
    int16_t	m[9];

    mpu6050->getMotions6(m);

    if (hmc5883L) {
	m[6] = hmc5883L->getMagnitudeX();
	m[7] = hmc5883L->getMagnitudeY();
	m[8] = hmc5883L->getMagnitudeZ();
    } else {
	m[6] = 0;
	m[7] = 0;
	m[8] = 0;
    }

    comp.calibrateItem(m);
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

timer_t calibrate_timer(int freq) {
    struct itimerspec	its;
    timer_t		timerid;
    struct sigevent	sev;

    memset((void *) &sev, 0, sizeof(sev));

    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = &calibrate_work;

    timer_create(CLOCK_REALTIME, &sev, &timerid);

    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 1000000000 / freq;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;

    timer_settime(timerid, 0, &its, NULL);

    return timerid;
}

void calibration() {
    timer_t	id = calibrate_timer(500);
    double	timeout = time_ns() + 5.0;

    std::cout << "Calibration: run" << std::endl;
    comp.clearCalibration();

    while (true) {
	double gx, gy, gz, gr;
	double ax, ay, az, ar;

	comp.getGyroSigma(gx, gy, gz, gr);
	comp.getAccelSigma(ax, ay, az, ar);

	if (comp.calcGyro(120.0)) {
	    imu.setAccelSigma(ar * 3.0);

	    printf(
		"Gyro:  %6.3f\t%6.3f\t%6.3f\t%6.2f\t%6.3f\t%6.3f\t%6.3f\n", 
		gx, gy, gz, gr, 
		comp.g_offset[0], comp.g_offset[1], comp.g_offset[2]
	    );

	    timeout = time_ns() + 5.0;
	}

	if (comp.calcAccel(100.0, 10.0)) {
	    printf(
		"Accel: %6.3f\t%6.3f\t%6.3f\t\t%6.3f\t%6.3f\t%6.3f\n",
		comp.a_offset[0], comp.a_offset[1], comp.a_offset[2],
		comp.a_scale[0], comp.a_scale[1], comp.a_scale[2]
	    );

	    timeout = time_ns() + 5.0;
	}

	if (comp.calcMag()) {
	    printf(
		"Mag:   %6.3f\t%6.3f\t%6.3f\t\t%6.3f\t%6.3f\t%6.3f\n",
		comp.m_offset[0], comp.m_offset[1], comp.m_offset[2],
		comp.m_scale[0], comp.m_scale[1], comp.m_scale[2]
	    );

	    timeout = time_ns() + 5.0;
	}

	double now = time_ns();

	if (now > timeout && comp.calibrated()) break;

	usleep(1000000/10);
    }

    timer_delete(id);
    control.storeConfig("imu.json");
    std::cout << "Calibration: done" << std::endl;
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
    } else {
        exit(1);
    }

    I2cPort *i2c1 = new I2cPort(0x1E, 0);
    i2c1->openConnection();

    #if 1
    if (i2c1->isConnectionOpen()) {
	hmc5883L = new HMC5883L(i2c1);

        hmc5883L->initialize();
        hmc5883L->setOutputRate(OUTPUT_RATE_6);	// 75Hz
    } else {
	exit(1);
    }
    #endif

    if (!comp.calibrated()) {
	calibration();
    }

    gyro_timer();

    if (hmc5883L) {
	mag_timer();
    }

    imu.setAccelSigma(0.002);
    control.work();

    return 0;
}
