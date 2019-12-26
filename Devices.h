#ifndef DEVICES_H
#define DEVICES_H

#include <inttypes.h>
#include <Eigen/Dense>

using namespace Eigen;

class Magnetometer {
public:
    virtual void getMagnitude(Vector3i &res) const = 0;
};

class Gyroscop {
public:
    virtual void getAngularVelocity(Vector3i &res) const = 0;
};

class Accelerometer {
public:
    virtual void getAcceleration(Vector3i &res) const = 0;
};

class Barometer {
};

#endif
