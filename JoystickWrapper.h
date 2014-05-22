/*
 * Wrapper for Joystick.
 * We are currently not using this file
 * Button Timeout
 * - Wait liitle bit after press button
 * 
 * Axis Filter
 * - Return 0 when input is less than 0.2
 */
#ifndef _JOYSTICKWRAPPER_H
#define _JOYSTICKWRAPPER_H

#include "WPILib.h"

#define JOYSTICK_TIMEOUT 0.1

class JoystickWrapper {
private:
	Joystick* m_gamepad;
	Timer* m_timer;
	
	float adjust (float);
	void trackTimer();
public:
	JoystickWrapper (Joystick*);
	JoystickWrapper (int);
	
	bool GetRawButton (int);
	bool GetButtonPress (int);
	float GetRawAxis (int);
};

#endif
