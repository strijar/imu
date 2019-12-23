#include <fstream>
#include <iostream>

#include <math.h>
#include "Compensation.h"

CalibrationItem::CalibrationItem() {
    r = 0;

    for (int i = 0; i < 3; i++) {
	a[i] = 0;
	g[i] = 0;
	m[i] = 0;
    }
}

void CalibrationItem::set(int16_t m[9]) {
    r = sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);

    for (int i = 0; i < 3; i++) {
	this->a[i] = m[i];
	this->g[i] = m[i+3];
	this->m[i] = m[i+6];
    }
}

///

Compensation::Compensation() {
    for (int i = 0; i < 3; i++) {
	a_offset[i] = 0.0;
	a_scale[i] = 1.0;
	g_offset[i] = 0.0;
	m_offset[i] = 0.0;
	m_scale[i] = 1.0;
    }

    itemIndex = 0;
    aValid = false;
    gValid = false;
    mValid = false;
}

void Compensation::calibrateItem(int16_t m[9]) {
    items[itemIndex].set(m);

    itemIndex = (itemIndex + 1) % itemsNum;
}

void Compensation::clearCalibration() {
    for (int i = 0; i < 3; i++) {
	aMin[i] = 0;
	aMax[i] = 0;
	mMin[i] = 0;
	mMax[i] = 0;
    }
}

bool Compensation::calibrated() {
    return gValid && aValid && mValid;
}

void Compensation::calcGyroAvr() {
    for (int i = 0; i < 3; i++)
	gAvr[i] = 0;

    for (int i = 0; i < itemsNum; i++)
	for (int n = 0; n < 3; n++)
	    gAvr[n] += items[i].g[n];

    for (int n = 0; n < 3; n++)
	gAvr[n] /= itemsNum;
}

void Compensation::calcMagAvr() {
    for (int i = 0; i < 3; i++)
	mAvr[i] = 0;

    for (int i = 0; i < itemsNum; i++)
	for (int n = 0; n < 3; n++)
	    mAvr[n] += items[i].m[n];

    for (int n = 0; n < 3; n++)
	mAvr[n] /= itemsNum;
}

void Compensation::calcAccelAvr() {
    for (int i = 0; i < 3; i++) {
	aAvr[i] = 0;
	aNorm[i] = 0;
    }

    for (int i = 0; i < itemsNum; i++)
	for (int n = 0; n < 3; n++) {
	    aAvr[n] += items[i].a[n];
	    aNorm[n] += items[i].a[n] / items[i].r;
	}

    for (int n = 0; n < 3; n++) {
	aAvr[n] /= itemsNum;
	aNorm[n] /= itemsNum;
    }
}

void Compensation::getGyroSigma(double &x, double &y, double &z, double &r) {
    calcGyroAvr();

    double sum[3] = { 0, 0, 0 };

    for (int i = 0; i < itemsNum; i++)
	for (int n = 0; n < 3; n++) {
	    double delta = gAvr[n] - items[i].g[n];

	    sum[n] += delta * delta;
	}

    x = sum[0] / itemsNum;
    y = sum[1] / itemsNum;
    z = sum[2] / itemsNum;

    r = sqrt(x + y + z);

    x = sqrt(x);
    y = sqrt(y);
    z = sqrt(z);
}

void Compensation::getAccelSigma(double &x, double &y, double &z, double &r) {
    calcAccelAvr();

    double sum[3] = { 0, 0, 0 };

    for (int i = 0; i < itemsNum; i++)
	for (int n = 0; n < 3; n++) {
	    double delta = aAvr[n] - items[i].a[n];

	    sum[n] += delta * delta;
	}

    x = sum[0] / itemsNum;
    y = sum[1] / itemsNum;
    z = sum[2] / itemsNum;

    r = sqrt(x + y + z);

    x = sqrt(x);
    y = sqrt(y);
    z = sqrt(z);
}

void Compensation::getAccelNorm(double &x, double &y, double &z) {
    x = aNorm[0];
    y = aNorm[1];
    z = aNorm[2];
}

bool Compensation::calcGyro(double noise) {
    double x, y, z, r;

    if (noise < gNoise)
	gNoise = noise;

    getGyroSigma(x, y, z, r);

    if (r != 0 && r < gNoise) {
	for (int i = 0; i < 3; i++)
	    g_offset[i] = gAvr[i];

	gNoise = r;
	gValid = true;
	return true;
    }

    return false;
}

bool Compensation::calcAccel(double noise, double angle) {
    double	x, y, z, r;
    double	e = cos(angle * 3.141526 / 180.0);
    bool	valid = true;

    getAccelSigma(x, y, z, r);

    if (r == 0) {
	return false;
    }

    if (r > aNoise) {
	if (r > aNoise * 100) {
	    aNoise = noise;
	    printf("reset\n");
	}
	return false;
    }

    for (int i = 0; i < 3; i++) {
	if (aNorm[i] > e) {
	    aMax[i] = aAvr[i];
	} else if (aNorm[i] < -e) {
	    aMin[i] = aAvr[i];
	}
    }

    for (int i = 0; i < 3; i++) {
	a_offset[i] = (aMax[i] + aMin[i]) / 2.0;
	a_scale[i] = (aMax[i] - aMin[i]) / 32768.0;

	valid = valid && aMax[i] != 0 && aMin[i] != 0;
    }

    aNoise = r;
    aValid = valid;
    return true;
}

bool Compensation::calcMag() {
    bool	valid = true;
    bool	ok = false;

    calcMagAvr();

    for (int n = 0; n < 3; n++) {
	double x = mAvr[n];

	if (x > mMax[n]) {
	    mMax[n] = x;
	    ok = true;
	}

	if (x < mMin[n]) {
	    mMin[n] = x;
	    ok = true;
	}
    }

    for (int i = 0; i < 3; i++) {
	m_offset[i] = (mMax[i] + mMin[i]) / 2.0;
	m_scale[i] = (mMax[i] - mMin[i]) / 1024.0;

	valid = valid && mMax[i] != 0 && mMin[i] != 0;
    }

    mValid = valid;
    return ok;
}

void Compensation::doIt(int16_t m[9], double d[9]) {
    for (int i = 0; i < 3; i++)
	d[i] = (m[i] - a_offset[i]) / a_scale[i];

    for (int i = 0; i < 3; i++)
	d[i+3] = m[i+3] - g_offset[i];

    for (int i = 0; i < 3; i++)
	d[i+6] = (m[i+6] - m_offset[i]) / m_scale[i];
}

void Compensation::loadAccel(json_value& args) {
    try {
	auto j_offset = args["offset"];
	auto j_scale = args["scale"];

	if (!j_offset.is_array()) {
	    aValid = false;
	    return;
	}

	if (!j_scale.is_array()) {
	    aValid = false;
	    return;
	}

	for (int i = 0; i < 3; i++) {
	    a_offset[i] = j_offset.as_array()[i].as_real();
	    a_scale[i] = j_scale.as_array()[i].as_real();
	}

	aValid = true;
    } catch (json_error const &e) {
	std::cout << e.what() << std::endl;
	aValid = false;
    }
}

void Compensation::loadMag(json_value& args) {
    try {
	auto j_offset = args["offset"];
	auto j_scale = args["scale"];

	if (!j_offset.is_array()) {
	    mValid = false;
	    return;
	}

	if (!j_scale.is_array()) {
	    mValid = false;
	    return;
	}

	for (int i = 0; i < 3; i++) {
	    m_offset[i] = j_offset.as_array()[i].as_real();
	    m_scale[i] = j_scale.as_array()[i].as_real();
	}

	mValid = true;
    } catch (json_error const &e) {
	std::cout << e.what() << std::endl;
	mValid = false;
    }
}

void Compensation::loadGyro(json_value& args) {
    if (!args.is_array()) {
	gValid = false;
	return;
    }

    try {
	for (int i = 0; i < 3; i++) {
	    g_offset[i] = args.as_array()[i].as_real();
	}
    } catch (json_error const &e) {
	std::cout << e.what() << std::endl;
	gValid = false;
    }

    gValid = true;
}

void Compensation::storeAccel(json_object& args) {
    json_array offset;
    json_array scale;

    for (int i = 0; i < 3; i++) {
	offset.push_back(a_offset[i]);
	scale.push_back(a_scale[i]);
    }

    args["offset"] = offset;
    args["scale"] = scale;
}

void Compensation::storeMag(json_object& args) {
    json_array offset;
    json_array scale;

    for (int i = 0; i < 3; i++) {
	offset.push_back(m_offset[i]);
	scale.push_back(m_scale[i]);
    }

    args["offset"] = offset;
    args["scale"] = scale;
}

void Compensation::storeGyro(json_array& args) {
    for (int i = 0; i < 3; i++) {
	args.push_back(g_offset[i]);
    }
}
