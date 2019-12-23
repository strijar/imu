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

	if (localSession->is_open()) {
	    std::cout << "Local session: open\n";

	    while (true) {
		sleep(5);
	    }
	} else {
	    std::cout << "Local session: problem";
	}
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
    double	pitch, roll, yaw;

    ahrs->getAngles(&roll, &pitch, &yaw);

    json_object opts;

    opts["pitch"] = pitch * 180.0 / PI;
    opts["roll"] = roll * 180.0 / PI;
    opts["yaw"] = yaw * 180.0 / PI;

    if (localSession) {
	localSession->publish("angle", {}, {{ opts }}, {});
    }
}

void Control::publishAccel() {
    json_object opts;

    opts["x"] = ahrs->a0 * 1000.0;
    opts["y"] = ahrs->a1 * 1000.0;
    opts["z"] = ahrs->a2 * 1000.0;

    if (localSession) {
	localSession->publish("accel", {}, {{ opts }}, {});
    }
}

void Control::work() {
    while (true) {
	publishAngle();
	publishAccel();

	usleep(1000000/publishFreq);
    }
}
