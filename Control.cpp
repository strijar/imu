#include <iostream>
#include <fstream>
#include <wampcc/json.h>
#include <unistd.h>

#include "Control.h"

#define PI 3.141526

using namespace wampcc;

Control::Control(Compensation *comp, MadgwickAHRS *ahrs) {
    this->comp = comp;
    this->ahrs = ahrs;
    this->mag = NULL;

    localThread = NULL;
    localSession = NULL;

    port = 55555;
    router = std::make_shared<wamp_router>(&theKernel);
}

Control::~Control() {
    if (localThread) {
	delete localThread;
    }
}

void Control::setMagnetometer(Magnetometer *mag) {
    this->mag = mag;
}

void Control::init() {
    auto fut = router->listen(auth_provider::no_auth_required(), port);

    if (auto ec = fut.get())
	throw std::runtime_error(ec.message());

    localThread = new std::thread(&Control::localWork, this);
    localThread->detach();
}

void Control::localWork() {
    std::unique_ptr<tcp_socket> 	socket(new tcp_socket(&theKernel));

    socket->connect("localhost", port).wait_for(std::chrono::seconds(3));

    if (socket->is_connected()) {
        std::cout << "Local session: connected\n";

	localSession = wamp_session::create<websocket_protocol>(&theKernel, std::move(socket));

	localSession->hello("imu").wait_for(std::chrono::seconds(3));

	while (localSession->is_open()) {
	    work();
	    usleep(1000000/publishFreq);
	}

	std::cout << "Local session: closed";
    }
}

bool Control::loadConfig(std::string filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
	config = json_value::make_object();
	return false;
    }

    std::ifstream::pos_type	pos = file.tellg();
    int				length = pos;
    char*			buf = new char[length];

    file.seekg(0, std::ios::beg);
    file.read(buf, length);
    file.close();

    try {
	config = json_decode(buf, length);

	comp->loadAccel(config["accel"]);
	comp->loadMag(config["mag"]);
	comp->loadGyro(config["gyro"]);

	delete buf;
	return true;
    } catch (json_error const &e) {
	std::cout << e.what() << std::endl;

	delete buf;
	return false;
    }

}

bool Control::storeConfig(std::string filename) {
    json_object	accel;
    json_object	mag;
    json_array	gyro;

    comp->storeAccel(accel);
    comp->storeMag(mag);
    comp->storeGyro(gyro);

    config["accel"] = accel;
    config["mag"] = mag;
    config["gyro"] = gyro;

    std::ofstream file(filename, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
	return false;
    }

    file << json_encode(config) << std::endl;
    file.close();

    return true;
}

void Control::publishAngle() {
    Vector3d	angle;

    ahrs->getAngles(angle);

    json_object opts;

    opts["pitch"] = angle(0) * 180.0 / PI;
    opts["roll"] = angle(1) * 180.0 / PI;
    opts["yaw"] = angle(2) * 180.0 / PI;

    localSession->publish("angle", {}, {{ opts }}, {});
}

void Control::publishAccel() {
    json_object opts;

    opts["x"] = ahrs->accel(0) * 1000.0;
    opts["y"] = ahrs->accel(1) * 1000.0;
    opts["z"] = ahrs->accel(2) * 1000.0;

    localSession->publish("accel", {}, {{ opts }}, {});
}

void Control::work() {
    publishAngle();
    publishAccel();
}

void Control::publishCalibrateAccel() {
}

void Control::publishCalibrateGyro() {
}

void Control::publishCalibrateMag() {
/*
    json_object opts, raw, cal;

    if (mag && localSession) {
	int16_t	m[3];
	double	c[3];

	comp->calcMagAvr();

	m[0] = comp->mAvr[0];	m[1] = comp->mAvr[1];	m[2] = comp->mAvr[2];

	comp->doMag(m, c);

	raw["x"] = m[0];	raw["y"] = m[1];	raw["z"] = m[2];
	cal["x"] = c[0];	cal["y"] = c[1];	cal["z"] = c[2];

	opts["raw"] = raw;
	opts["cal"] = cal;

	localSession->publish("calMag", {}, {{ opts }}, {});
    }
*/
}
