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
    this->accel = NULL;
    this->gyro = NULL;

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

void Control::setMagnetometer(Vector3i *mag) {
    this->mag = mag;
}

void Control::setGyroscop(Vector3i *gyro) {
    this->gyro = gyro;
}

void Control::setAccelerometer(Vector3i *accel) {
    this->accel = accel;
}

void Control::init() {
    auto fut = router->listen(auth_provider::no_auth_required(), port);

    if (auto ec = fut.get())
	throw std::runtime_error(ec.message());

    router->callable("imu", "GetFreq",
	[this](wamp_router&, wamp_session& caller, call_info info) { getFreq(caller, info); }
    );

    router->callable("imu", "SetFreq",
	[this](wamp_router&, wamp_session& caller, call_info info) { setFreq(caller, info); }
    );

    router->callable("imu", "LoadConfig",
	[this](wamp_router&, wamp_session& caller, call_info info) { loadConfig(caller, info); }
    );

    router->callable("imu", "StoreConfig",
	[this](wamp_router&, wamp_session& caller, call_info info) { storeConfig(caller, info); }
    );

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

void Control::getFreq(wamp_session& caller, call_info& info) {
    json_object			answer;

    answer["ok"] = true;
    answer["freq"] = publishFreq;

    caller.result(info.request_id, {}, answer);
}

void Control::setFreq(wamp_session& caller, call_info& info) {
    json_object			answer;
    const json_object		args = info.args.args_dict;
    auto			freq = args.find("freq");

    if (freq == args.end()) {
	answer["ok"] = false;
	answer["error"] = "Need freq";
    } else {
	publishFreq = freq->second.as_uint();
	answer["ok"] = true;
    }

    caller.result(info.request_id, {}, answer);
}

void Control::loadConfig(wamp_session& caller, call_info& info) {
    json_object			answer;
    const json_object		args = info.args.args_dict;
    auto			name = args.find("name");

    if (name == args.end()) {
	answer["ok"] = false;
	answer["error"] = "Need name";
    } else {
	if (loadConfig(name->second.as_string())) {
	    answer["ok"] = true;
	} else {
	    answer["ok"] = false;
	    answer["error"] = "File problem";
	}
    }

    caller.result(info.request_id, {}, answer);
}

void Control::storeConfig(wamp_session& caller, call_info& info) {
    json_object			answer;
    const json_object		args = info.args.args_dict;
    auto			name = args.find("name");

    if (name == args.end()) {
	answer["ok"] = false;
	answer["error"] = "Need name";
    } else {
	if (storeConfig(name->second.as_string())) {
	    answer["ok"] = true;
	} else {
	    answer["ok"] = false;
	    answer["error"] = "File problem";
	}
    }

    caller.result(info.request_id, {}, answer);
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

    publishAccelRaw();
    publishGyroRaw();
    publishMagRaw();
}

void Control::publishAccelRaw() {
    json_object	opts;

    opts["x"] = (*accel)(0);
    opts["y"] = (*accel)(1);
    opts["z"] = (*accel)(2);

    localSession->publish("accel.raw", {}, {{ opts }}, {});
}

void Control::publishGyroRaw() {
    json_object	opts;

    opts["x"] = (*gyro)(0);
    opts["y"] = (*gyro)(1);
    opts["z"] = (*gyro)(2);

    localSession->publish("gyro.raw", {}, {{ opts }}, {});
}

void Control::publishMagRaw() {
    json_object	opts;

    opts["x"] = (*mag)(0);
    opts["y"] = (*mag)(1);
    opts["z"] = (*mag)(2);

    localSession->publish("mag.raw", {}, {{ opts }}, {});
}
