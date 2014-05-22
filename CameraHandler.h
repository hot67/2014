
#ifndef _CAMERAHANDLER_H
#define _CAMERAHANDLER_H

#include "WPILib.h"
	#include <cmath>
	#include "nivision.h"
	#include "Defines.h"


	class CameraHandler	
	{
	public:
		
		CameraHandler(AxisCamera *camera, DriverStationLCD *m_dsLCD, Relay *relay);
		
		enum state_t {
			kNone,
			kLeft,
			kRight,
			kError
		};
		
		// Returns the x-position of vision target in 1.0 to -1.0
		// 0.0 means the vision target is the center of the picture (just infront of the robot)
		double getCenter();
		
		// Returns what goal is hot
		// - kLeft, kRight, kNone, kError
		state_t getHotGoal();
		
		// Returns if the left goal is hot goal
		// We may not need to use it because we already have getHotGoal function
		bool getLeftHot();
		
		// Same with getLeftHot
		bool getRightHot();
		
		// Get Ball
		double getBallX();
		double GetDistanceToBall();
		
	private:
		
		AxisCamera *camera;
		DriverStation *m_ds;
		DriverStationLCD *m_dsLCD;
		ColorImage *img;
		ColorImage *img2;
		Relay *light;
	};
	
#endif
