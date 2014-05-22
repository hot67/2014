/*
 * We Currently not using this file
 */

#ifndef ARMWRAPPER_H
#define ARMWRAPPER_H

#include "WPILib.h"
#include <cmath>

class ArmWrite;

	class ArmWrapper: public PIDOutput {

	private:
		SpeedController* m_lArm;
		SpeedController* m_rArm;
		Encoder* m_armAngle;
		DigitalInput* m_armLimSwitch;

		// PID
		PIDController* PID;
		bool PIDFlag;
		
		// Config
		double c_distPerPulse;
		double c_maxPeriod;

		// Functions
		void mainConstructor (SpeedController* lArm, SpeedController* rArm, Encoder* armAngle, DigitalInput* armLimSwitch);

	public:
		// Constructor
		// (Left Arm, Right Arm, Encoder, Dist / Pulse, Max Period)
		ArmWrapper (SpeedController* lArm, SpeedController* rArm, Encoder* armAngle, DigitalInput* armLimSwitch);
		// (motor1channel, moter2channel, encoderchannel1, encoderChannel2, reverseDirection, distPerPulse, maxPeriod)
		ArmWrapper (int lArm, int rArm, int armAngle1, int armAngle2, int armLimSwitch);
		ArmWrapper (int lArm, int rArm, int armAngle1, int armAngle2, bool direction, int armLimSwitch);

		// Control The Motor
		void Set (float);

		// PID
		//void StartPID (float, float, float);
		void SetAngle (float);
		void PIDEnable ();
		void PIDDisable ();

		// For PID Controller use
		void PIDWrite (float);

		// Control Encoder
		void Reset ();
		void Start ();
		void Stop ();

		// Get Values
		int GetRawAngle ();
		double GetAngle ();
		double GetSpeed ();		// Returns in [Dist Unit]/[microsec]
		bool GetLimSwitch();
		double PIDGet ();
		double PIDOutput ();

		bool bGrabberSafty ();

		// Conf
		void SetDistPerPulse (double);
		void SetMaxPeriod (double);
		double GetDistPerPulse ();
		double GetMaxPeriod ();
	};
	
	class ArmWrite: public PIDOutput
	{
	public:
		ArmWrite(SpeedController* output);
		
		void PIDWrite (float output);
		
		float GetMaxThrottle ();
		float GetLastValue ();
		void ChangeMaxThrottle (float deltaThrottle);
		
		void Reset();
	private:
		SpeedController* m_arm;
		float m_lastValue;
		float m_maxThrottle;
	};

#endif
