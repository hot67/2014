
#ifndef _DEFINES_H
#define _DEFINES_H

// Buttons

#define BUTTON_A 1
#define BUTTON_B 2
#define BUTTON_X 3
#define BUTTON_Y 4
#define BUTTON_LB 5
#define BUTTON_RB 6
#define BUTTON_BACK 7
#define BUTTON_START 8
#define BUTTON_L3 9
#define BUTTON_R3 10

// Axes

#define LEFT_X 1
#define LEFT_Y 2
#define TRIGGERS 3
#define RIGHT_X 4
#define RIGHT_Y 5
#define ACCEL_CAP 0.1

//Universal constants
#define CAMERA_ANGLE 0.82030475

//#define PracticeBot
#define CompetitionBot


#ifdef PracticeBot
	// Place practice bot values here
	#define FLOOR_PICKING_POS 378.0
	#define MED_SHOOT_POS 115.0
	#define MED_SHOT_BACK -198.0 //-175.0
	#define GUARDED_SHOT_FRONT 90.0
	#define GUARDED_SHOT_BACK -130.5
	#define LONG_SHOOT_POS -126
	#define CATCH_POS 14.0
	#define TRUSS_SHOT 49.0
	#define BGRABBER_SAFE 3
	#define AUTON_SHOOT_POS 1
	#define SELF_CATCH -31.0

	#define ARM_RESET_WIDTH 68.0

	#define AUTON_DRIVE_FORWARD_DIST 200
	#define AUTON_DRIVE_BACK_DIST 30
	
	#define AUTON_ANGLE_GAP 10


	// PID Config for PID
	#define ARM_P 0.01
	#define ARM_I 0.0
	#define ARM_D 0.0

	// Drive PID
	#define DRV_P 0.1
	#define DRV_I 0.000
	#define DRV_D 0.0
	#define REV_IN 100.0

	// RAMROD
	#define RAM_LOCK_POSITION 930
	#define RAM_MID_POSITION 450

	// Drive Rotate
	#define CAMERA_VIEW 1
	#define DEGREE_FACTOR 26.5
	#define ANGLE_TO_HOTGOAL 28.13
	#define ROTATE_ANGLE_GAP 5

	//Special PWMs
	#define ROLLER_PWM 8
#endif

#ifdef CompetitionBot
	// Place bot values here
	#define FLOOR_PICKING_POS 381
	#define MED_SHOOT_POS 135
	#define MED_SHOT_BACK -188.0
	#define GUARDED_SHOT_FRONT 104.0
	#define GUARDED_SHOT 50
	#define GUARDED_SHOT_BACK -132.5
	#define SELF_CATCH -15.0
	#define TRUSS_SHOT 49.0
	#define LONG_SHOOT_POS -141
	#define CATCH_POS -1.0
	#define BGRABBER_SAFE 3
	#define AUTON_SHOOT_POS 1

	#define ARM_RESET_WIDTH 83.0

	#define AUTON_DRIVE_FORWARD_DIST 200
	#define AUTON_DRIVE_BACK_DIST 30
	
	#define AUTON_ANGLE_GAP 10


	// PID Config for PID
	#define ARM_P 0.01
	#define ARM_I 0.0
	#define ARM_D 0.0

	// Drive PID
	#define DRV_P 0.0625
	#define DRV_I 0.000
	#define DRV_D 0.210
	#define REV_IN 100.0

	#define AUTONDRV_STRAIGHT_P 0.15
	#define AUTONDRV_STRAIGHT_I 0.00
	#define AUTONDRV_STRAIGHT_D 0.210
	
	#define AUTONDRV_ROTATE_P 0.300
	#define AUTONDRV_ROTATE_I 0.000
	#define AUTONDRV_ROTATE_D 0.000

	// RAMROD
	#define RAM_LOCK_POSITION 930
	#define RAM_MID_POSITION 350

	// Drive Rotate
	#define CAMERA_VIEW 1
	#define DEGREE_FACTOR 26.5
	#define ANGLE_TO_HOTGOAL 28.13
	#define ROTATE_ANGLE_GAP 5

	//Special PWMs
	#define ROLLER_PWM 8
#endif

#endif
