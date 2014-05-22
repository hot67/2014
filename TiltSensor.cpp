
#include "TiltSensor.h"
#include <cmath>

void TiltSensor::InitTiltSensor()
{
	m_angle = GetNewAccelAngle();

	m_deltaT->Start();
	m_deltaT->Reset();
}

TiltSensor::TiltSensor(ADXL345_I2C* accelerometer, Gyro* gyro, double tau)
{
	m_accelerometer = accelerometer;
	m_gyro = gyro;
	m_tau = tau;	// response time

	m_deltaT = new Timer();

	InitTiltSensor();
}

TiltSensor::~TiltSensor()
{
	delete m_deltaT;
}

double TiltSensor::GetNewAccelAngle()
{
	double accelerationAngle;
	double accelerationX = m_accelerometer->GetAcceleration(ADXL345_I2C::kAxis_X);
	double accelerationY = m_accelerometer->GetAcceleration(ADXL345_I2C::kAxis_Y);

	if(fabs(accelerationY) < 0.001) {
		accelerationAngle = 0.0;
	}
	else {
		accelerationAngle = atan2(accelerationX, accelerationY);
	}

	return accelerationAngle;
}

double TiltSensor::GetNewGyroRate()
{
	return m_gyro->GetRate()*3.14159/180.0;
}

double TiltSensor::UpdateAngle()
{
	double dT = m_deltaT->Get();
	m_deltaT->Reset();

	m_alpha = m_tau / (m_tau + dT);

	m_angle = m_alpha * (m_angle + GetNewGyroRate() * dT) + (1.0 - m_alpha) * (GetNewAccelAngle());

	return m_angle;
}

double TiltSensor::GetAngle()
{
	m_angle = UpdateAngle();
	return m_angle;
}

double TiltSensor::PIDGet()
{
	return GetAngle();
}
