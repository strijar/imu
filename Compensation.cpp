#include <fstream>
#include <iostream>

#include <math.h>
#include "Compensation.h"

Compensation::Compensation() {
    accelOffset = Vector3d(0.0, 0.0, 0.0);
    accelScale = Vector3d(1.0, 1.0, 1.0);

    gyroOffset = Vector3d(0.0, 0.0, 0.0);

    magOffset = Vector3d(0.0, 0.0, 0.0);

    magTrans = Matrix3d::Zero();
    magTrans.setIdentity();
}

void Compensation::doAccel(Vector3i src, Vector3d &dst) {
    dst = src.cast<double>() - accelOffset;

    dst(0) *= accelScale(0);
    dst(1) *= accelScale(1);
    dst(2) *= accelScale(2);
}

void Compensation::doGyro(Vector3i src, Vector3d &dst) {
    dst = src.cast<double>() - gyroOffset;
}

void Compensation::doMag(Vector3i src, Vector3d &dst) {
    dst = magTrans * (src.cast<double>() - magOffset);
}

void Compensation::loadAccel(json_value& args) {
    try {
	auto j_offset = args["offset"];
	auto j_scale = args["scale"];

	if (!j_offset.is_array()) return;
	if (!j_scale.is_array()) return;

	for (int i = 0; i < 3; i++) {
	    accelOffset(i) = j_offset.as_array()[i].as_real();
	    accelScale(i) = j_scale.as_array()[i].as_real();
	}
    } catch (json_error const &e) {
	std::cout << e.what() << std::endl;
    }
}

void Compensation::loadMag(json_value& args) {
    try {
	auto j_offset = args["offset"];
	auto j_trans = args["trans"];

	if (!j_offset.is_array()) return;
	if (!j_trans.is_array()) return;

	for (int i = 0; i < 3; i++) {
	    magOffset(i) = j_offset.as_array()[i].as_real();

	    for (int n = 0; n < 3; n++)
		magTrans(i, n) = j_trans.as_array()[i].as_array()[n].as_real();
	}
    } catch (json_error const &e) {
	std::cout << e.what() << std::endl;
    }
}

void Compensation::loadGyro(json_value& args) {
    if (!args.is_array()) return;

    try {
	for (int i = 0; i < 3; i++) {
	    gyroOffset(i) = args.as_array()[i].as_real();
	}
    } catch (json_error const &e) {
	std::cout << e.what() << std::endl;
    }
}

void Compensation::storeAccel(json_object& args) {
/*
    json_array offset;
    json_array scale;

    for (int i = 0; i < 3; i++) {
	offset.push_back(a_offset[i]);
	scale.push_back(a_scale[i]);
    }

    args["offset"] = offset;
    args["scale"] = scale;
*/
}

void Compensation::storeMag(json_object& args) {
}

void Compensation::storeGyro(json_array& args) {
}
