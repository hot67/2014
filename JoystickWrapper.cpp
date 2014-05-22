#include "JoystickWrapper.h"

JoystickWrapper::JoystickWrapper (Joystick* gamepad) {
	m_gamepad = gamepad;
	m_timer = new Timer;
	
	m_timer->Reset();
}

JoystickWrapper::JoystickWrapper(int gamepad) {
	m_gamepad = new Joystick (gamepad);
	m_timer = new Timer;
	
	m_timer->Reset();
}

float JoystickWrapper::adjust (float input) {
	if (input < -0.2) {
		return input;
	} else if (input < 0.2) {
		return 0.0;
	} else {
		return input;
	}
}

void JoystickWrapper::trackTimer () {
	if (m_timer->HasPeriodPassed(JOYSTICK_TIMEOUT))
	{
		m_timer->Stop();
		m_timer->Reset();
	}
}

bool JoystickWrapper::GetRawButton (int channel) {
	return m_gamepad->GetRawButton (channel);
}

bool JoystickWrapper::GetButtonPress (int channel) {
	trackTimer();
	
	if (m_gamepad->GetRawButton(channel) && (m_timer->Get() == 0.0))
	{
		m_timer->Start();
		return true;
	}
	else
		return false;
}

float JoystickWrapper::GetRawAxis (int channel) {
	return adjust(m_gamepad->GetRawAxis (channel));
}
