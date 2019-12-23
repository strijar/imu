//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

class MadgwickAHRS {
private:

    double beta;		// algorithm gain
    double q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
    double aSigma;

    double invSqrt(double x);

public:
    double a0, a1, a2;		// acceleration
    double v0, v1, v2;		// velocity;
    double x, y, z;		// pos;

public:
    MadgwickAHRS(double beta);

    void update(double dt, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
    void updateIMU(double dt, double gx, double gy, double gz, double ax, double ay, double az);
    void getAngles(double *roll, double *pitch, double *yaw);
    void gravityCompensate(double ax, double ay, double az);
    void integrate(double dt);

    void setAccelSigma(double x);
};

#endif
