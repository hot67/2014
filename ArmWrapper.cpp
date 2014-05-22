#include "WPILib.h"
#include "Defines.h"
#include "ArmWrapper.h"


// ----- Constructor -----

// With Object
ArmWrapper::ArmWrapper (SpeedController* lArm, SpeedController* rArm, Encoder* armAngle, DigitalInput* armLimSwitch) {
	m_lArm = lArm;
	// m_rArm = rArm;
	m_armAngle = armAngle;
	m_armLimSwitch = armLimSwitch;

	// ----- Start PID -----
	PID = new PIDController (ARM_P, ARM_I, ARM_D, m_armAngle, this);
	PIDFlag = false;
	
	// ----- Set Conf -----
	c_distPerPulse = 1.0;
	c_maxPeriod = 1.0;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
	m_armAngle->SetMaxPeriod(c_maxPeriod);
}

// With Channel
ArmWrapper::ArmWrapper(int lArm, int rArm, int armAngle1, int armAngle2, int armLimSwitch) {
	m_lArm = new Talon(lArm);
	// m_rArm = new Talon(rArm);
	m_armAngle = new Encoder(armAngle1, armAngle2, true);
	m_armLimSwitch = new DigitalInput(armLimSwitch);

	// ----- Start PID -----
	PID = new PIDController (ARM_P, ARM_I, ARM_D, m_armAngle, this);
	PIDFlag = false;
	
	// ----- Set Conf -----
	c_distPerPulse = 1.0;
	c_maxPeriod = 1.0;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
	m_armAngle->SetMaxPeriod(c_maxPeriod);
	m_armAngle->Start();
}

// With Channel and reverse direction
ArmWrapper::ArmWrapper(int lArm, int rArm, int armAngle1, int armAngle2, bool reverse, int armLimSwitch) {
	m_lArm = new Talon(lArm);
	// m_rArm = new Talon(rArm);
	m_armAngle = new Encoder(armAngle1, armAngle2, reverse);
	m_armLimSwitch = new DigitalInput(armLimSwitch);

	// ----- Start PID -----
	PID = new PIDController (ARM_P, ARM_I, ARM_D, m_armAngle, this);
	PIDFlag = false;
	
	// ----- Set Conf -----
	c_distPerPulse = 1.0;
	c_maxPeriod = 1.0;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
	m_armAngle->SetMaxPeriod(c_maxPeriod);
}

// ----- Control Motors -----
void ArmWrapper::Set (float speed) {
	m_lArm->Set (-speed);
	// m_rArm->Set (speed);
}

// ----- PID -----
void ArmWrapper::SetAngle (float angle) {
	PID->SetSetpoint(angle);
}

void ArmWrapper::PIDEnable () {
	PID->Enable();
	PIDFlag = true;
}

void ArmWrapper::PIDDisable () {
	if (PIDFlag) {
		PID->Disable();
		PIDFlag = false;
	}
}

void ArmWrapper::PIDWrite(float output) {
	Set(output);
}

// ----- Control Encoder -----
void ArmWrapper::Reset() {
	m_armAngle->Reset();
}

void ArmWrapper::Start() {
	m_armAngle->Start();
}

void ArmWrapper::Stop () {
	m_armAngle->Stop();
}

// ----- Get Values -----
int ArmWrapper::GetRawAngle () {
	return m_armAngle->Get();
}

double ArmWrapper::GetAngle () {
	return (double)m_armAngle->GetDistance();
}

double ArmWrapper::GetSpeed() {
	return m_armAngle->GetRate();
}

bool ArmWrapper::GetLimSwitch() {
	return m_armLimSwitch->Get();
}

bool ArmWrapper::bGrabberSafty () {
	return GetRawAngle() < BGRABBER_SAFE;
}

double ArmWrapper::PIDGet () {
	return m_armAngle->PIDGet();
}

double ArmWrapper::PIDOutput () {
	return PID->Get();
}

// ----- Conf -----
void ArmWrapper::SetDistPerPulse(double distPerPulse) {
	c_distPerPulse = distPerPulse;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
}

void ArmWrapper::SetMaxPeriod(double maxPeriod) {
	c_maxPeriod = maxPeriod;
	m_armAngle->SetMaxPeriod(c_maxPeriod);
}

double ArmWrapper::GetDistPerPulse () {
	return c_distPerPulse;
}

double ArmWrapper::GetMaxPeriod () {
	return c_maxPeriod;
}

//ARM WRITE WRAPPER****************************

ArmWrite::ArmWrite (SpeedController* output)
{
	m_arm = output;
	m_maxThrottle = 0.2;
	m_lastValue = 0;
}

void ArmWrite::PIDWrite (float output)
{
	if (m_lastValue > 1)
		m_arm->Set(output);
	else {
		m_arm->Set(output * m_lastValue);
		m_lastValue += m_maxThrottle;
	}
}

float ArmWrite::GetLastValue ()
{
	return m_lastValue;
}

float ArmWrite::GetMaxThrottle ()
{
	return m_maxThrottle;
}

void ArmWrite::ChangeMaxThrottle (float deltaThrottle)
{
	m_maxThrottle += deltaThrottle;
}

void ArmWrite::Reset ()
{
	m_lastValue = 0;
}
