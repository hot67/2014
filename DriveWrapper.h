
#ifndef _DRIVEWRAPPER_H
#define _DRIVEWRAPPER_H

#include "WPILib.h"
#include "Defines.h"
#include <cmath>


class DriveAuton;
class DriveStraightSource;
class DriveRotateSource;

class DriveWrapper : public SpeedController {
public:
	//Constructor
	DriveWrapper(SpeedController*, SpeedController*);
	
	void Set(float,uint8_t);
	void Set(float,float,uint8_t);
	float Get();
	void Disable();
	void PIDWrite(float);
	
	void Rotate(double speed, double angle);
	void RatateEnable();
	bool IsRotating();
	
private:
	SpeedController* m_drive1;
	SpeedController* m_drive2;
	
	
};

class DriveStraightSource : public PIDSource, public PIDOutput
{
public:
	DriveStraightSource (DriveAuton *p);
	
	void Set(double dist);
	void Enable();
	void Disable();
	bool IsEnabled();
	double GetSetPoint();
	bool IsFinished();
	
	void SetPID(float p, float i, float d);
	float GetP();
	float GetI();
	float GetD();
		
	
	double PIDGet();
	void PIDWrite(float output);
private:
	PIDController* PID;
	DriveAuton* parent;
};

class DriveRotateSource : public PIDSource, public PIDOutput
{
public:
	DriveRotateSource (DriveAuton *p);
	
	void Set(double angle);
	void Enable();
	void Disable();
	bool IsEnabled();
	double GetSetPoint();
	bool IsFinished();
	
	void SetPID(float p, float i, float d);
	float GetP();
	float GetI();
	float GetD();
		
	
	double PIDGet();
	void PIDWrite(float output);
private:
	PIDController* PID;
	DriveAuton *parent;
};

class DriveAuton {
public:
	DriveAuton (RobotDrive* robotDrive, Encoder* lEncoder, Encoder* rEncoder);
	
	friend class DriveStraightSource;
	friend class DriveRotateSource;
	
	void Set(double dist, double angle);
	void Enable();
	void Disable();
	bool IsEnabled();
	double GetDist();
	double GetAngle();
	double GetSetDist();
	double GetSetAngle();
	bool IsFinished();
	bool IsDriveFinished();
	bool IsRotateFinished();
	
	void SetRotatePID(float p, float i, float d);
	void SetStraightPID(float p, float i, float d);
	float GetRotateP();
	float GetRotateI();
	float GetRotateD();
	float GetStraightP();
	float GetStraightI();
	float GetStraightD();
	
protected:
	// ----- Components -----
	RobotDrive* m_robotDrive;
	Encoder* m_lEncoder;
	Encoder* m_rEncoder;
	
	// ----- Values -----
	double m_straightVal;
	double m_rotateVal;
private:
	// ----- Dummy Objects -----
	DriveStraightSource *m_dStraight;
	DriveRotateSource *m_dRotate;
	
	// ----- Flags -----
	bool f_enabled;
	bool f_setted;
	
};

class DriveStraightPID: public PIDSource, public PIDOutput
{
public:
	DriveStraightPID (RobotDrive* roboDrv, Encoder* leftEncode, Encoder* rightEncode);
	
	double PIDGet ();
	void PIDWrite (float output);
	
private:
	RobotDrive* m_drive;
	Encoder* m_lEncode;
	Encoder* m_rEncode;
	
	double m_pidOut;
};

class DriveRotate: public PIDSource, public PIDOutput
{
public:
	DriveRotate (RobotDrive* robotDrive, Encoder* lEncoder, Encoder* rEncoder);
	
	void SetAngle (double forwardspeed, double forwarddistance, double angle);
	void PIDEnable ();
	void PIDDisable ();
	bool PIDIsEnabled ();
	bool IsRotating (double gap);
	bool IsRotating ();
	bool Finished();

	void PIDWrite(float input);
	double PIDGet();
private:
	// ----- Components -----
	RobotDrive* m_robotDrive;
	Encoder* m_lEncoder;
	Encoder* m_rEncoder;
	
	// ----- PID -----
	PIDController* PID;
	
	// ----- Values -----
	double m_forwardspeed, m_forwarddistance;
	
	
	
	// ----- Flags -----
	// TO Avoid initializing ini_angle in loop
	bool f_angleInitialized;
	bool PIDFlag;
	// Check the direction of rotation
	enum {kLeft, kRight} f_turningDirection;
};

#endif //DRIVEWRAPPER_H
