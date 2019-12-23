#ifndef COMPENSATION_H
#define COMPENSATION_H

#include <wampcc/json.h>
#include <inttypes.h>

using namespace wampcc;

class CalibrationItem {
public:
    double	r = 0.0;
    double	a[3];
    double	g[3];
    double	m[3];

    CalibrationItem();

    void set(int16_t m[9]);
};

class Compensation {
private:

    static const int	itemsNum = 500/4;

    CalibrationItem	items[itemsNum];
    int			itemIndex;

    double		gAvr[3];
    double		gNoise = 100.0;
    bool		gValid = false;

    double		aNorm[3];
    double		aMin[3];
    double		aMax[3];
    double		aNoise = 100.0;
    bool		aValid = false;

    double		mAvr[3];
    double		mMin[3];
    double		mMax[3];
    bool		mValid = false;

    void calcGyroAvr();
    void calcAccelAvr();
    void calcMagAvr();

public:
    double		aAvr[3];

    double		a_offset[3];
    double		a_scale[3];
    double		g_offset[3];
    double		m_offset[3];
    double		m_scale[3];

    Compensation();

    void calibrateItem(int16_t m[9]);

    void clearCalibration();
    bool calibrated();

    void getGyroSigma(double &x, double &y, double &z, double &r);
    bool calcGyro(double noise);

    void getAccelSigma(double &x, double &y, double &z, double &r);
    void getAccelNorm(double &x, double &y, double &z);
    bool calcAccel(double noise, double angle);

    bool calcMag();

    void doIt(int16_t m[9], double d[9]);

    void loadAccel(json_value& args);
    void loadMag(json_value& args);
    void loadGyro(json_value& args);

    void storeAccel(json_object& args);
    void storeMag(json_object& args);
    void storeGyro(json_array& args);
};

#endif
