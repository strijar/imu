#ifndef COMPENSATION_H
#define COMPENSATION_H

#include <Eigen/Dense>
#include <wampcc/json.h>
#include <inttypes.h>

using namespace wampcc;
using namespace Eigen;

class Compensation {
private:

    Vector3d		accelOffset;
    Vector3d		accelScale;

    Vector3d		gyroOffset;

    Vector3d		magOffset;
    Matrix3d		magTrans;

public:

    Compensation();

    void doAccel(Vector3i src, Vector3d &dst);
    void doGyro(Vector3i src, Vector3d &dst);
    void doMag(Vector3i src, Vector3d &dst);

    bool loadAccel(json_value& args);
    bool loadMag(json_value& args);
    bool loadGyro(json_value& args);

    void storeAccel(json_object& args);
    void storeMag(json_object& args);
    void storeGyro(json_array& args);
};

#endif
