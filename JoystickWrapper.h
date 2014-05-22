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
