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
    Magnetometer	*mag;

    void localWork();

    void publishAngle();
    void publishAccel();

    void work();

public:
    Control(Compensation *comp, MadgwickAHRS *ahrs);
    virtual ~Control();

    void setMagnetometer(Magnetometer *mag);

    void init();

    bool loadConfig(std::string filename);
    bool storeConfig(std::string filename);

    void publishCalibrateAccel();
    void publishCalibrateGyro();
    void publishCalibrateMag();

};

#endif
