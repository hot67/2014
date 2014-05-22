/*
 * We are currently not using this file
 */
#ifndef _TILTSENSOR_H_
#define _TILTSENSOR_H_

#include "PIDSource.h"
#include "ADXL345_I2C.h"
#include "Gyro.h"
#include "Timer.h"

class TiltSensor : public PIDSource
{
public:
	TiltSensor(ADXL345_I2C* accelerometer, Gyro* gyro, double tau);
	virtual ~TiltSensor();

	double UpdateAngle();
	double GetAngle();
	double PIDGet();

private:
	void InitTiltSensor();
	double GetNewAccelAngle();
	double GetNewGyroRate();

	ADXL345_I2C* m_accelerometer;
	Gyro* m_gyro;
	double m_tau;		// response time
	double m_alpha;		// = tau / (tau + dT)

	Timer *m_deltaT;
	double m_angle;
};
#endif
