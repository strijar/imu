#ifndef CONTROL_H
#define CONTROL_H

#include <wampcc/wampcc.h>
#include <wampcc/json.h>

#include "Devices.h"
#include "Compensation.h"
#include "MadgwickAHRS.h"

using namespace wampcc;

class Control {
private:
    int					port;
    kernel				theKernel;
    std::shared_ptr<wamp_router>	router;
    std::thread*			localThread;
    std::shared_ptr<wamp_session>	localSession;
    int					publishFreq = 10;

    json_value	config;

    Compensation	*comp;
    MadgwickAHRS	*ahrs;

    Vector3i		*mag;
    Vector3i		*gyro;
    Vector3i		*accel;

    void localWork();

    void publishAngle();
    void publishAccel();

    void publishAccelRaw();
    void publishGyroRaw();
    void publishMagRaw();

    void work();

    void getFreq(wamp_session& caller, call_info& info);
    void setFreq(wamp_session& caller, call_info& info);
    void loadConfig(wamp_session& caller, call_info& info);
    void storeConfig(wamp_session& caller, call_info& info);

public:
    Control(Compensation *comp, MadgwickAHRS *ahrs);
    virtual ~Control();

    void setMagnetometer(Vector3i *mag);
    void setGyroscop(Vector3i *gyro);
    void setAccelerometer(Vector3i *accel);

    void init();

    bool loadConfig(std::string filename);
    bool storeConfig(std::string filename);

    void publishCalibrateAccel();
    void publishCalibrateGyro();
    void publishCalibrateMag();

};

#endif
