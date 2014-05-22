#include "WPILib.h"
#include "JoystickWrapper.h"
#include "DriveWrapper.h"
// #include "ArmWrapper.h"
// #include "CameraHandler.h"
#include "CheesyVisionServer.h"
#include "Defines.h"
#include <cmath>

/**
 * HOTBOT 2014 v2.9 - Build Date: 4/5/14
 * 
 * See the wiki on the GitHub repository for more information
 * 
 * Controller Map:
 * 
 * Teleop
 * ======
 * 
 * Driver
 * ------
 * 
 * Left Analog Up/Down: Drive Forward/Backward
 * Right Analog Left/Right: Move Left/Right
 * 
 * Right Bumper (HOLD): Shift Down
 * 
 * Left Trigger: Medium-power shot
 * Right Trigger: Full-power shot
 * 
 * Operator
 * --------
 * 
 * Left Analog Up/Down: Manual arm manipulation
 * 
 * A: Arm at floor pickup position
 * B: Arm at medium shot position
 * X: Arm at long shot position
 * Y: Arm at catch position
 * 
 * Back: Toggle catch arms
 * Start: Toggle bottom arm
 * 
 * Left Trigger: Roller backwards
 * Right Trigger: Roller forwards
 * 
 * Test
 * ====
 * 
 * Driver
 * ------
 * 
 * Left Analog Up/Down: Drive Forward/Backward
 * Right Analog Left/Right: Move Left/Right
 * 
 * Left Analog Push: Reset left encoder
 * Right Analog Push: Reset right encoder
 * 
 * A: Shifter
 * 
 * Left Bumper: Ramrod In
 * Right Bumper: Ramrod Out
 * 
 * Right Trigger: Release ramrod
 * Left Trigger: Lock ramrod
 * 
 * Start: Reset ramrod encoder
 * 
 * Operator
 * --------
 * 
 * Left Analog Up/Down: Arm control
 * 
 * Left Analog Push: Reset arm encoder
 * 
 * A: Extend catch arms
 * B: Retract catch arms
 * X: Grab with bottom arm
 * Y: Release with bottom arm
 * 
 * Start: Camera light
 * 
 * Left trigger: Roll out
 * Right trigger: Roll in
 * 
 * Other useful information
 * ========================
 * Left trigger: TRIGGERS > 0
 * Right trigger: TRIGGERS < 0
 */

enum AutonChoice {
	AutonDBrebound,
	AutonDFshoot,
	AutonCheckHotleft,
	AutonCheckHotright,
	AutonTwoBallTwohot,
	AutonTurnleft,
	AutonTurnright,
	AutonDoNothing,
	AutonDf,
	AutonBalltrack
};

class BuiltinDefaultCode: public IterativeRobot {
private:
	// MOTOR CONTROLLERS *********************************

	//Declare drive motors
	Talon* m_lDrive1; //Two motors
	Talon* m_rDrive1; //One motor
	Talon* m_lDrive2; //Two motors
	Talon* m_rDrive2; //One motor

	//Declare arm
	Talon* m_armMotor;

	//Declare ramrod motor
	Talon *m_ramMotor;

	//Declare bGrabber motor
	Talon* m_roller;

	// SERVOS ********************************************

	//Declare ramrod servo
	Servo *m_ramServo;

	//Declare catch arm stop servos
	Servo *m_catchServo1;
	Servo *m_catchServo2;

	//Declare camera light
	Relay* m_camLight;

	// PNEUMATICS ****************************************

	//Declare Compressor
	Compressor* m_compressor;

	// shifters
	Solenoid *m_shifters;

	//Declare bGrabber solenoids
	Solenoid* m_catch;
	Solenoid* m_bArm;

	// DRIVE ABSTRACTION OBJECTS *************************

	//Declare drive objects
	DriveWrapper* m_rDrive;
	DriveWrapper* m_lDrive;
	RobotDrive* m_robotDrive;
	DriveRotate* m_driveRotate;
	DriveStraightPID* m_drvSource;
	DriveAuton* m_autonDrive;

	// SENSORS *******************************************

	//Drivetrain Encodes
	Encoder *m_rEncode;
	Encoder *m_lEncode;

	//Arm encoder
	Encoder* m_armEncoder;

	//Declare ramrod encoder
	Encoder *m_ramEncode;

	//Arm light sensor
	DigitalInput* m_armReset;

	// PID CONTROLLERS ***********************************

	//Arm PID controller
	PIDController* m_armPID;

	//Drive PID controller
	PIDController* m_drvStraightPID;

	// OTHER ABSTRACTION OBJECTS *************************

	ArmWrapper* m_arm;
	ArmWrite* m_armWrite;

	//Declare camera handler object


	// DRIVER INTERFACE OBJECTS **************************

	//Declare joysticks
	JoystickWrapper* m_driver;
	JoystickWrapper* m_operator;

	//Declare driver station
	DriverStation* m_ds;
	DriverStationLCD* m_dsLCD;

	// MISCELLANEOUS *************************************

	//Timers
	Timer *m_ramTime;
	Timer *m_rollTime;
	Timer *m_autonTime;

	// Counter
	int countLoop;
	int armCount;
	int autonCount;

	int ramFire;
	int m_medRamCase;
	int m_ramCase;
	bool m_ramInit;
	bool m_autonShot; //bool to determine whether we have shot or not
	bool m_rollIn;
	bool m_dunRollin;
	bool m_armPIDFlag;
	float m_armDOffset;
	double m_currentVal;
	//double aCCEL_CAP;
	//Auton Selector Variables
	AutonChoice autonChoice;
	Timer* m_selectorCountdown;
	int m_selectorPage;

	//PrintData Disable Override
	bool m_printDataInMatch;

	//Arm reset variables
	double m_armResetPos;
	double m_armOffset;
	bool m_armResetFlag;
	bool m_canResetArm;
	bool m_armResetStop;
	bool m_shiftOverride;

	// Auton Steps
	int AutonDBSteps;
	int AutonSteps;
	int autondance;

	//Current Sensor
	AnalogChannel *m_currentSensor;
	Timer *m_currentTimer;

	CheesyVisionServer *m_cheesyVisionServer;

public:

	/**
	 * Constructor for this "BuiltinDefaultCode" Class.
	 * 
	 * The constructor creates all of the objects used for the different inputs and outputs of
	 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
	 * providing named objects for each of the robot interfaces. 
	 */

	BuiltinDefaultCode() {
		// MOTOR CONTROLLERS *********************************

		//Initialze drive controllers
		m_rDrive1 = new Talon(1);
		m_rDrive2 = new Talon(2);
		m_lDrive1 = new Talon(3);
		m_lDrive2 = new Talon(4);

		//Initialize ramrod motor
		m_ramMotor = new Talon(5);

		//Initialize relays
		m_camLight = new Relay(2);

		//Initialize Arm
		m_armMotor = new Talon(6);

		//initialize bGrabber motor
		m_roller = new Talon(7);

		// SERVOS ********************************************

		//Initialize ramrod servo
		m_ramServo = new Servo(10);

#ifdef CompetitionBot
		//Initialize catch servos
		m_catchServo1 = new Servo(8);
		m_catchServo2 = new Servo(9);
#endif

		// SENSORS *******************************************

		//Drive encoders
		m_rEncode = new Encoder(1, 2, true);
		m_rEncode->SetDistancePerPulse(1);
		m_rEncode->SetMaxPeriod(1.0);
		m_rEncode->Start();

		m_lEncode = new Encoder(3, 4, false);
		m_lEncode->SetDistancePerPulse(1);
		m_lEncode->SetMaxPeriod(1.0);
		m_lEncode->Start();

		//Initialize the arm encoder
		m_armEncoder = new Encoder(5, 6, false);
		//m_armEncoder = new Encoder (5, 6, true);
		m_armEncoder->SetDistancePerPulse(1.0);
		m_armEncoder->SetMaxPeriod(1.0);
		m_armEncoder->SetReverseDirection(false);
		m_armEncoder->Start();

		//Initialize ramrod encoder
		m_ramEncode = new Encoder(7, 8, true);
		m_ramEncode->SetDistancePerPulse(1);
		m_ramEncode->SetMaxPeriod(1.0);
		m_ramEncode->Start();

		//Arm reset light sensor
		m_armReset = new DigitalInput(11);

		// PNEUMATICS ****************************************

		//Initialize Compressor
		m_compressor = new Compressor(9, 1);

		//shifters
		m_shifters = new Solenoid(1);

		//Initialize bGrabber Solenoids
		m_bArm = new Solenoid(2);
		m_catch = new Solenoid(3);

		// DRIVE ABSTRACTION OBJECTS *************************

		//Initialize drive wrappers
		m_rDrive = new DriveWrapper(m_rDrive1, m_rDrive2);
		m_lDrive = new DriveWrapper(m_lDrive1, m_lDrive2);

		//Initialize robot drive
		m_robotDrive = new RobotDrive(m_lDrive, m_rDrive);
		m_robotDrive->SetSafetyEnabled(false);

		//Initialize Drive Rotate
		m_driveRotate = new DriveRotate(m_robotDrive, m_lEncode, m_rEncode);
		m_drvSource = new DriveStraightPID(m_robotDrive, m_lEncode, m_rEncode);
		m_autonDrive = new DriveAuton(m_robotDrive, m_lEncode, m_rEncode);

		// OTHER ABSTRACTION OBJECTS *************************

		m_arm = new ArmWrapper(6, 8, 5, 6, 10);
		m_armWrite = new ArmWrite(m_armMotor);

		// PID CONTROLLERS ***********************************

		m_armPID = new PIDController(ARM_P, ARM_I, ARM_D, m_armEncoder,
				m_armWrite);
		m_drvStraightPID = new PIDController(DRV_P, DRV_I, DRV_D, m_drvSource,
				m_drvSource);

		// DRIVER INTERFACE OBJECTS **************************

		//Initialize joysticks
		m_driver = new JoystickWrapper(1);
		m_operator = new JoystickWrapper(2);

		//Initialize camera handler object
		//Initialize camera handler object


		//Grab driver station object
		m_ds = DriverStation::GetInstance();
		m_dsLCD = DriverStationLCD::GetInstance();

		// MISCELLANEOUS *************************************

		//Timers
		m_ramTime = new Timer;
		m_autonTime = new Timer;
		m_rollTime = new Timer;

		m_ramCase = -1;
		m_medRamCase = -1;
		countLoop = 0;
		armCount = 0;
		m_rollIn = false;
		m_autonShot = false;
		m_dunRollin = true;

		m_armDOffset = 0.0;

		//Arm reset variables
		m_armResetPos = 0;
		m_armOffset = 0.0;
		m_canResetArm = false;
		m_armResetFlag = false;
		m_armResetStop = false;
		m_shiftOverride = false;
		m_currentVal = 0;
		//ACCEL_CAP = 0;

		//Auton Selector Variables
		m_selectorCountdown = new Timer;
		m_selectorPage = 0;

		//Print Data Disable Override
		m_printDataInMatch = false;

		// Auton Steps
		AutonDBSteps = 1;
		AutonSteps = 0;
		autondance = 0;

		//Current Sensor
		m_currentSensor = new AnalogChannel(1);
		m_currentTimer = new Timer;

		m_cheesyVisionServer = CheesyVisionServer::GetInstance();
	}

	/********************************** Init Routines *************************************/

	void RobotInit() {
		m_cheesyVisionServer->SetPort(1180);
		m_cheesyVisionServer->StartListening();

		// m_cheesyVisionServer->Reset();
		// m_cheesyVisionServer->StartSamplingCounts();
	}

	void DisabledInit() {
		autonChoice = AutonDFshoot;
		m_cheesyVisionServer->StopSamplingCounts();
	}

	void AutonomousInit() {
		// Auton Steps
		AutonSteps = 0;
		AutonDBSteps = 1;
		m_armEncoder->Reset();
		m_ramEncode->Reset();
		m_rEncode->Reset();
		m_lEncode->Reset();
		m_ramCase = -1;
		m_medRamCase = -1;

		// Make Thread for CheesyVision
		m_cheesyVisionServer->Reset();
		m_cheesyVisionServer->StartSamplingCounts();

		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
				"      AUTONOMOUS     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
				"         Mode:       ");

		switch (autonChoice) {
		case AutonDFshoot:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"       DF Shoot      ");
			break;
		case AutonDf:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"     Drive Forward   ");
			break;
		case AutonDBrebound:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"        2 Ball       ");
			break;
		case AutonDoNothing:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"       DISABLED      ");
			break;
		case AutonTurnleft:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"      Turn Left      ");
			break;
		case AutonTurnright:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"      Turn Right     ");
			break;
		case AutonCheckHotleft:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"    Check Hot Left   ");
			break;
		case AutonCheckHotright:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"    Check Hot Right  ");
			break;
		case AutonTwoBallTwohot:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"      2-Ball Hot     ");
			break;
		case AutonBalltrack:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"     Ball Tracking   ");
		}

		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
				"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
				"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
				"                     ");
		m_dsLCD->UpdateLCD();
	}

	void TeleopInit() {
		m_shifters -> Set(false);
		m_ramCase = -1;
		m_medRamCase = -1;
		m_ramInit = false;
		m_ramTime->Stop();
		m_ramTime->Start();
		m_ramTime->Reset();
		m_rEncode->Reset();
		m_lEncode->Reset();
		//m_armEncoder->Reset();
		m_armPID->Disable();
		m_armPID->SetPID(ARM_P, ARM_I, ARM_D);
		m_armPID->Reset();
		m_armWrite->Reset();
		AutonDBSteps = 0;
		m_robotDrive->TankDrive(0.,0.);
		m_drvStraightPID->Disable();

		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
				"       TELEOP        ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
				"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
				"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
				"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
				"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
				"                     ");
		m_dsLCD->UpdateLCD();

		// SmartDashboard::PutBoolean("Left Goal Hot (Count): ", m_cheesyVisionServer->GetLeftCount() > 20);
		// SmartDashboard::PutBoolean("Right Goal Hot (Count): ", m_cheesyVisionServer->GetRightCount() > 20);

		//m_cheesyVisionServer->Reset();
		m_cheesyVisionServer->StopSamplingCounts();

	}

	void TestInit() {
		AutonDBSteps = 0;

		m_dsLCD->Clear();
		m_dsLCD->UpdateLCD();
	}

	/********************************** Periodic Routines *************************************/
	void DisabledPeriodic() {

		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
				"HOTBOT b.4-05-14 v2.9");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
				"  ||   ||  __  ----- ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
				"  ||--|| /    \\   |  ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
				"  ||   || \\__/   |   ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
				"        Auton:       ");

		if (m_operator->GetRawButton(BUTTON_A)) {
			autonChoice = AutonDFshoot;
			m_selectorPage = 0;
		} else if (m_operator->GetRawButton(BUTTON_B)) {
			autonChoice = AutonTwoBallTwohot;
			m_selectorPage = 0;
		} else if (m_operator->GetRawButton(BUTTON_X)
				&& m_operator->GetRawButton(BUTTON_LB)) {
			autonChoice = AutonCheckHotright;
			m_selectorPage = 0;
		} else if (m_operator->GetRawButton(BUTTON_X)) {
			autonChoice = AutonCheckHotleft;
			m_selectorPage = 0;
		} else if (m_operator->GetRawButton(BUTTON_Y)) {
			autonChoice = AutonDBrebound;
			m_selectorPage = 0;
		} else if (m_operator->GetRawButton(BUTTON_RB)) {
			autonChoice = AutonDf;
			m_selectorPage = 0;
		}
		/*else if (m_operator->GetRawButton(BUTTON_LB))
		 {
		 autonChoice = AutonBalltrack;
		 m_selectorPage = 0;
		 }*/
		else if (m_operator->GetRawButton(BUTTON_START)) {
			if (m_selectorPage == 1)
				m_selectorPage = 0;
			else
				m_selectorPage++;
		}

		if (m_driver->GetRawButton(BUTTON_BACK))
			m_printDataInMatch = !m_printDataInMatch;

		switch (autonChoice) {
		case AutonDBrebound:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"       2 Ball       ");
			break;
		case AutonDFshoot:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"      DF Shoot       ");
			break;
		case AutonCheckHotleft:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"   Check Left Hot    ");
			break;
		case AutonCheckHotright:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"   Check Right Hot   ");
			break;
		case AutonTwoBallTwohot:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"      2-Ball Hot     ");
			break;
		case AutonTurnright:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"      Turn Right     ");
			break;
		case AutonTurnleft:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"      Turn Left      ");
			break;
		case AutonDoNothing:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"      DISABLED       ");
			break;
		case AutonDf:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"    Drive Forward    ");
			break;
		case AutonBalltrack:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"     Ball Tracker    ");
			break;
		}

		if (m_printDataInMatch) {
			m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
					"   DEBUG MODE ENABLED  ");
		}

		if (m_operator->GetRawButton(BUTTON_BACK) && autonChoice
				!= AutonDoNothing) {
			if (m_selectorCountdown->Get() > 2.0) {
				m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
						"DISABLING...RELEASE");
			} else {
				if (m_selectorCountdown->Get() == 0.0)
					m_selectorCountdown->Start();

				m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
						"                     ");
				m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
						"DISABLING...%f ",
						(float) (2.0 - m_selectorCountdown->Get()));
			}
			m_selectorPage = 0;
		} else if (m_selectorCountdown->HasPeriodPassed(2.0)) {
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"      DISABLED       ");
			autonChoice = AutonDoNothing;
			m_selectorCountdown->Stop();
			m_selectorCountdown->Reset();
		} else {
			m_selectorCountdown->Stop();
			m_selectorCountdown->Reset();
		}

		switch (m_selectorPage) {
		case 1:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
					"A: DF Shoot          ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"B: Check Hot Right   ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"X: Check Hot Left    ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
					"Y: 2 Ball            ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
					"RB: Drive Forward    ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"Back (HOLD): Disable ");
			break;
		case 2:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,
					"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
					"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
					"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,
					"                     ");
			break;
		}

		PrintData();
		m_dsLCD->UpdateLCD();
	}

	void AutonomousPeriodic() {
		ManageCompressor();
		ArmReset();
		WatchArm();
		PrintData();
		RamFire();

		// m_camLight->Set(Relay::kForward);

		switch (autonChoice) {
		case AutonDf:
			AutonDF();
			break;
		case AutonDFshoot:
			AutonDFShoot();
			break;
		case AutonDBrebound:
			AutonDBReboundRun();
			break;
		case AutonTurnleft:
			AutonTurnLeft();
			break;
		case AutonTurnright:
			AutonTurnRight();
			break;
		case AutonCheckHotleft:
			AutonCheckHotLeft();
			break;
		case AutonCheckHotright:
			AutonCheckHotRight();
			break;
		case AutonTwoBallTwohot:
			AutonTwoBallTwoHot();
			break;
			/*
			 case AutonBalltrack:
			 //Ball tracker auton
			 break;
			 */
		}

		if (m_armPID->IsEnabled() && fabs(
				m_armEncoder->GetDistance() - m_armPID->GetSetpoint()) < 15
				&& m_armPID->GetSetpoint() != MED_SHOT_BACK + 1) {
			m_armPID->SetPID(ARM_P, 0.003, ARM_D);
		} else {
			m_armPID->SetPID(ARM_P, ARM_I, ARM_D);
		}
	}

	void TeleopPeriodic() {
		ManageCompressor();
		TeleopDrive();
		RamrodInit();
		RamFire();
		MedRamFire();
		RamrodOverride();
		TeleopArm();
		WatchArm();
		ArmReset();
		//AutoArmReset();
		TeleopBGrabber();
		AutoDownShift();
		PrintData();
		// m_camLight->Set(Relay::kForward);

		// SmartDashboard::PutBoolean("Left Goal Hot (Count): ", m_cheesyVisionServer->GetLeftCount() > 20);
		// SmartDashboard::PutBoolean("Right Goal Hot (Count): ", m_cheesyVisionServer->GetRightCount() > 20);


		/*
		 if(m_driver->GetRawButton(BUTTON_A))
		 {
		 SmartDashboard::PutNumber("Hot Goal Detection: ", m_cameraHandler->getHotGoal());
		 }
		 if (m_driver->GetRawButton(BUTTON_B))
		 {
		 ColorImage* pImage = m_camera->GetImage();
		 if (pImage)
		 {
		 if (pImage->GetHeight() == 0 || pImage->GetWidth()==0)
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"Image size error"); 
		 else
		 {
		 pImage->Write("Image.jpg");
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"Image wrote"); 
		 }
		 
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5,"Width: %f",(float)pImage->GetWidth());
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line6,"Height: %f",(float)pImage->GetHeight());

		 delete pImage;
		 }
		 else
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"Image was null"); 
		 }
		 else
		 printf("Never got the camera instance. Big error.\n");
		 //SmartDashboard::PutNumber("Hot Goal Detection: ", m_cameraHandler->getHotGoal());
		 m_dsLCD->UpdateLCD();
		 
		 */
		//SmartDashboard::PutBoolean("Left Hot: ", m_cheesyVisionServer->GetLeftStatus());
		//SmartDashboard::PutNumber("Left Count: ", m_cheesyVisionServer->GetLeftCount());
		//SmartDashboard::PutBoolean("Right Hot: ", m_cheesyVisionServer->GetRightStatus());
		//SmartDashboard::PutNumber("Right Count: ", m_cheesyVisionServer->GetLeftCount());
	}

	void TestPeriodic() {
		ManageCompressor();
		TestArm();
		TestDrive();
		TestBGrabber();
		TestRamMotion();
		TestRamLock();
		TestFindSensorWidth();
		PrintData();

		// ----- Camera Test -----
		m_camLight->Set(Relay::kForward);
		/*
		 if (m_driver->GetRawButton(BUTTON_Y))
		 {
		 // camera->WriteResolution(AxisCamera::kResolution_320x240);
		 
		 if (m_camera)
		 {
		 ColorImage* pImage = m_camera->GetImage();
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,"GetImage: %s",(bool)pImage ? "yes" : "no");
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,"Width: %f", (float)pImage->GetWidth());
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4,"Height: %f",(float)pImage->GetHeight());
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5,"Resolution: %s",m_camera->GetResolution() == AxisCameraParams::kResolution_320x240 ? "yes" : "no");
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5,"Image Status: %s",pImage->StatusIsFatal() ? "false" : "true");
		 }
		 }
		 
		 if (camera)
		 {
		 ColorImage* pImage = camera->GetImage();
		 if (pImage)
		 {
		 if (pImage->GetHeight() == 0 || pImage->GetWidth()==0)
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"Image size error"); 
		 else
		 {
		 pImage->Write("Image.jpg");
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"Image wrote"); 
		 }
		 
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5,"Width: %f",(float)pImage->GetWidth());
		 m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line6,"Height: %f",(float)pImage->GetHeight());

		 delete pImage;
		 }
		 else
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"Image was null"); 
		 }
		 else
		 printf("Never got the camera instance. Big error.\n");
		 //SmartDashboard::PutNumber("Hot Goal Detection: ", m_cameraHandler->getHotGoal());
		 m_dsLCD->UpdateLCD();
		 }
		 */
	}

	/******************************** TEST ***************************/
	void TeleopTest() {
		SmartDashboard::PutBoolean("Is Finished: ", m_autonDrive->IsFinished());
		SmartDashboard::PutBoolean("Is Enabled: ", m_autonDrive->IsEnabled());
		SmartDashboard::PutNumber("Get Dist: ", m_autonDrive->GetDist());
		SmartDashboard::PutNumber("Get Set Dist: ", m_autonDrive->GetSetDist());
		SmartDashboard::PutNumber("Get Angle: ", m_autonDrive->GetAngle());
		SmartDashboard::PutNumber("Get Set Angle: ",
				m_autonDrive->GetSetAngle());

		if (m_driver->GetRawButton(BUTTON_A)) {
			m_autonDrive->Set(24.0, 0.0);
			m_autonDrive->Enable();
		} else if (m_driver->GetRawButton(BUTTON_B)) {
			m_shifters->Set(true);
			m_autonDrive->Set(0.0, 90.0);
			m_autonDrive->Enable();
		} else if (m_driver->GetRawButton(BUTTON_Y)) {
			m_shifters->Set(true);
			m_autonDrive->Set(48.0, 90.0);
			m_autonDrive->Enable();
		} else {
			m_autonDrive->Disable();
		}
	}

	/********************************** External Routines *************************************/

	/*********************** AUTONOMOUS FUNCTIONS ****************************/

	void AutonDBRebound() {
		switch (AutonDBSteps) {
		case 1:

			if (!m_armPID->IsEnabled()) {
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				m_armPID->Enable();
			}

			if (!m_drvStraightPID->IsEnabled()) {
				m_drvStraightPID->SetSetpoint(-64.0);
				m_drvStraightPID->Enable();
			}

			if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
					< 5) {
				AutonDBSteps++;
				m_drvStraightPID->Disable();
			}
			if (fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK)
					< AUTON_ANGLE_GAP && m_ramCase == -1) {
				m_ramCase = 0;
			}
			break;

		case 2:
			SmartDashboard::PutNumber("Arm Difference",
					fabs(m_armEncoder->GetDistance() - LONG_SHOOT_POS));
			if (fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK)
					< AUTON_ANGLE_GAP && m_ramCase == -1) {
				m_ramCase = 0;
			}

			if (m_ramCase > 2) {
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				AutonDBSteps++;
			}
			break;

		case 3:
			m_armPID->SetSetpoint(FLOOR_PICKING_POS);
			m_roller->Set(-1.0);
			if (m_autonTime->HasPeriodPassed(0.5)) {
				m_drvStraightPID->SetSetpoint(55.0);
				m_drvStraightPID->Enable();
			}
			SmartDashboard::PutNumber("Arm Difference",
					fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));

			if ((fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS)
					< AUTON_ANGLE_GAP) && fabs(
					m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
					< 5) {
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				m_drvStraightPID->Disable();
				AutonDBSteps++;
			}
			break;
		case 4:
			if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
					< 5 || m_autonTime->HasPeriodPassed(2.0)) {
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_drvStraightPID->Disable();
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				AutonDBSteps++;
			}

			if (m_autonTime->HasPeriodPassed(.5)) {
				m_autonTime->Stop();
				m_autonTime->Reset();
				m_drvStraightPID->SetSetpoint(40.0);
				m_drvStraightPID->Enable();
			}
			break;
		case 5:
			m_drvStraightPID->SetSetpoint(-105.0);
			m_drvStraightPID->Enable();

			SmartDashboard::PutNumber("Arm Difference",
					fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));

			if ((fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK)
					< AUTON_ANGLE_GAP)) {
				if (fabs(
						m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
						< 50 && m_ramCase == -1)
					m_ramCase = 0;
				else if (fabs(
						m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
						< 5) {
					m_rEncode -> Reset();
					m_lEncode -> Reset();
					m_drvStraightPID->Disable();
					m_autonTime->Stop();
					m_autonTime->Reset();
					m_roller->Set(0.0);
					//AutonDBSteps++;
					AutonDBSteps = 7;
				}
			}
			break;
		case 6:
			if (m_ramCase == -1)
				m_ramCase = 0;
			else if (m_ramCase == 3) {
				m_armPID->Disable();
				AutonDBSteps++;
			}
			break;
		case 7:
			/*
			 m_armPID->SetSetpoint(0.0);
			 m_armPID->Enable();
			 */
			break;

		}
	}

	void AutonTwoBallTwoHot() {
		static bool leftStatus;

		// We always shift down
		m_shifters->Set(true);
		m_autonDrive->SetRotatePID(AUTONDRV_ROTATE_P, AUTONDRV_ROTATE_I, AUTONDRV_ROTATE_D);

		switch (AutonSteps) {
		case 0: // Drive Forward, Move Arm, Detect Hotgoal

			if (!m_armPID->IsEnabled()) {
				m_armPID->SetSetpoint(MED_SHOT_BACK - 6);
				m_armPID->Enable();
			}

			m_autonDrive->Set(-64.0, 0.0);
			m_autonDrive->Enable();

			if (m_autonDrive->IsFinished()) {
				// Check is Left Status is Hot
				leftStatus = (m_cheesyVisionServer->GetLeftCount() > 10);

				m_autonTime->Stop();
				m_autonTime->Reset();
				m_autonTime->Start();
				m_autonDrive->Disable();
				AutonSteps++;
			}
			break;

		case 1: // Turn to Hotgoal, Shoot
			if (m_autonTime->Get() > 0.5) {
				if (leftStatus) {
					// Left is Hot
					m_autonDrive->Set(0.0, 30.0);
					m_autonDrive->Enable();
				} else {
					// Right is Hot
					m_autonDrive->Set(0.0, -30.0);
					m_autonDrive->Enable();
				}

				if (m_autonDrive->IsFinished()) {
					// Turning is End
					m_autonTime->Stop();
					m_autonTime->Reset();

					m_autonDrive->Disable();
					
					// Shoot
					m_roller->Set(0.0);
					m_ramCase = 0;	// Shoot: Do we need to wait?
					
					AutonSteps++;
				}
			}
			break;

		case 2: // Turn Back, Move Arm
			if (m_ramCase >= 4) {
				m_roller->Set(-1.0);
				m_armPID->SetSetpoint(FLOOR_PICKING_POS);

				if (leftStatus) {
					// Left Was Hot
					m_autonDrive->Set(0.0, -30.0);
					m_autonDrive->Enable();
				} else {
					// Right Was Hot
					m_autonDrive->Set(0.0, 30.0);
					m_autonDrive->Enable();
				}

				if (m_autonDrive->IsFinished()) {
					// Turning Back is End
					m_autonDrive->Disable();
					
					m_autonTime->Stop();
					m_autonTime->Reset();
					m_autonTime->Start();
					
					AutonSteps++;					
				}
			}

			break;

		case 3: // Drive Back, Move Roller
			if (m_autonTime->Get() > 0.5) {
				m_autonDrive->Set(55.0, 0.0);
				m_autonDrive->Enable();

				m_roller->Set(-1.0);

				if (m_autonDrive->IsFinished()) {
					
					m_autonDrive->Disable();
					
					m_autonTime->Stop();
					m_autonTime->Reset();
					m_autonTime->Start();
					
					AutonSteps++;
				}
			}
			break;

		case 4: // Second Move Back to Make Sure we got the Ball
			if (m_autonTime->Get() > 0.5) {
				m_autonDrive->Set(40.0, 0.0);
				m_autonDrive->Enable();

				m_roller->Set(-1.0);

				if (m_autonDrive->IsFinished()) {
					m_armPID->SetSetpoint(MED_SHOT_BACK);	// Need to wait? Shouldn't it be next case?
					
					m_autonTime->Stop();
					m_autonTime->Reset();
					m_autonTime->Start();

					m_autonDrive->Disable();

					AutonSteps++;
				}
			}

			break;

		case 5: // Move Foward, The Arm is moving
			if (m_autonTime->Get() > 0.5) {
				m_autonDrive->Set(-105.0, 0);
				m_autonDrive->Enable();

				m_roller->Set(-1.0);

				if (m_autonDrive->IsFinished()) {
					m_autonDrive->Disable();

					m_autonTime->Stop();
					m_autonTime->Reset();
					m_autonTime->Start();
					
					AutonSteps++;
				}
			}
			break;

		case 6:	// Turn to Hot
			if (m_autonTime->Get() > 0.5) {

				if (leftStatus) {
					m_autonDrive->Set(0.0, -30.0);
					m_autonDrive->Enable();
				} else {
					m_autonDrive->Set(0.0, 30.0);
					m_autonDrive->Enable();
				}

				if (m_autonDrive->IsFinished()) {
					m_autonTime->Stop();
					m_autonTime->Reset();

					m_autonDrive->Disable();
					m_roller->Set(0.0);
					m_ramCase = 0;
					AutonSteps++;
					AutonDBSteps = 7;
				}
			}
			break;

		case 7:
			// SmartDashboard::PutNumber("Auton Total Time: ", m_ds->GetMatchTime());
			AutonSteps++;
			break;

		}
	}

	void AutonDBReboundRun() {
		switch (AutonDBSteps) {
		case 1: // Move Arm, Drive Forward, Shoot

			if (!m_armPID->IsEnabled()) {
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				m_armPID->Enable();
			}

			if (!m_drvStraightPID->IsEnabled()) {
				m_drvStraightPID->SetSetpoint(-64.0);
				m_drvStraightPID->Enable();
			}

			if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())< 5) {
				AutonDBSteps++;
				m_drvStraightPID->Disable();
			}

			if (fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK) < AUTON_ANGLE_GAP && m_ramCase == -1) {
				m_ramCase = 0;
			}
			break;

		case 2: // Make sure we shoot
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - LONG_SHOOT_POS));
			if (fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK)
					< AUTON_ANGLE_GAP && m_ramCase == -1) {
				m_ramCase = 0;
			}

			if (m_ramCase > 2) {
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				m_armWrite->Reset();
				AutonDBSteps++;
			}
			break;

		case 3: // Move back
			m_armPID->SetSetpoint(FLOOR_PICKING_POS);
			m_roller->Set(-1.0);
			if (m_autonTime->HasPeriodPassed(0.5)) {
				m_drvStraightPID->SetSetpoint(55.0);
				m_drvStraightPID->Enable();
			}
			SmartDashboard::PutNumber("Arm Difference",fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));

			if ((fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS) < 20.0) && fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 5) {
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				m_drvStraightPID->Disable();
				AutonDBSteps++;
			}
			break;

		case 4: // Second Move back and get ball
			if (m_autonTime->HasPeriodPassed(.5)) {
				m_autonTime->Stop();
				m_autonTime->Reset();
				m_drvStraightPID->SetSetpoint(40.0);
				m_drvStraightPID->Enable();
			}

			if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
					< 5) {
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_drvStraightPID->Disable();
				m_armWrite->Reset();
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				AutonDBSteps++;
			}
			break;

		case 5: // Move Foward	
			m_drvStraightPID->SetSetpoint(-105.0);
			m_drvStraightPID->Enable();

			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));
			
			if (fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK) < AUTON_ANGLE_GAP) {
				if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 100 && m_ramCase == -1) {
					m_ramCase = 0;
					m_roller->Set(0.0);
				} else if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 5 && m_ramCase >= 4) {
					m_rEncode -> Reset();
					m_lEncode -> Reset();
					m_drvStraightPID->Disable();
					m_autonTime->Stop();
					m_autonTime->Reset();
					m_armPID->Disable();
					AutonDBSteps = 7;
				}
			}
			break;

		case 6: // Make sure we shoot
			if (m_ramCase == -1)
				m_ramCase = 0;
			else if (m_ramCase == 3) {
				m_armPID->Disable();
				AutonDBSteps++;
			}
			break;
		case 7:
			/*
			 m_armPID->SetSetpoint(0.0);
			 m_armPID->Enable();
			 */
			break;

		}
	}

	void AutonDFShoot() {

		AutonShotSafety();

		switch (AutonSteps) {
		case 0:

			if (!m_drvStraightPID->IsEnabled()) {
				m_drvStraightPID->SetSetpoint(-74.0);
				m_drvStraightPID->Enable();
			}

			if (!m_armPID->IsEnabled()) {
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				m_armPID->Enable();
			}

			if (fabs(MED_SHOT_BACK - m_armEncoder->GetDistance())
					< AUTON_ANGLE_GAP && fabs(
					m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
					< 5) {
				m_ramCase = 0;
				m_autonShot = true;
				AutonSteps++;
			}
			break;
		case 1:
			if (m_drvStraightPID->IsEnabled())
				m_drvStraightPID->Disable();
			if (m_armPID->IsEnabled())
				m_armPID->Disable();

			if (m_ramCase == 5) {
				AutonDBSteps = 7;
				AutonSteps++;
			}
			break;
		}
	}
	void AutonCheckHotLeft() {
		AutonShotSafety();

		switch (AutonSteps) {
		case 0:
			//m_autonDrive->Set(-64.0, 0.0);
			//m_autonDrive->Enable();
			if (!m_armPID->IsEnabled()) {
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				m_armPID->Enable();
			}

			if (!m_drvStraightPID->IsEnabled()) {
				m_drvStraightPID->SetSetpoint(-88.0);
				m_drvStraightPID->Enable();
			}

			if (m_cheesyVisionServer->GetLeftCount() > 10) {
				if (!m_autonShot)
					m_ramCase = 0;
				AutonSteps++;
				m_armPID->Disable();
				m_drvStraightPID->Disable();
				m_autonShot = true;
			}
			break;
		case 1:
			if (m_ramCase >= 3)
				AutonDBSteps = 7;
			break;
		}
	}

	void AutonCheckHotRight() {
		AutonShotSafety();

		switch (AutonSteps) {
		case 0:
			//m_autonDrive->Set(-64.0, 0.0);
			//m_autonDrive->Enable();
			if (!m_armPID->IsEnabled()) {
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				m_armPID->Enable();
			}

			if (!m_drvStraightPID->IsEnabled()) {
				m_drvStraightPID->SetSetpoint(-88.0);
				m_drvStraightPID->Enable();
			}
			if (m_cheesyVisionServer->GetRightCount() > 10) {
				if (!m_autonShot)
					m_ramCase = 0;
				AutonSteps++;
				m_armPID->Disable();
				m_drvStraightPID->Disable();
				m_autonShot = true;
			}
			break;
		case 1:
			if (m_ramCase >= 3)
				AutonDBSteps = 7;
			break;
		}
	}
	void AutonDF() {
		if (!m_drvStraightPID->IsEnabled()) {
			m_drvStraightPID->SetSetpoint(-64.0);
			m_drvStraightPID->Enable();
		}
	}

	void AutonTurnLeft() {
		switch (AutonSteps) {
		case 0:
			m_driveRotate->SetAngle(0, 0, 18);
			m_driveRotate->PIDEnable();

			m_armPID->SetSetpoint(MED_SHOT_BACK);
			m_armPID->Enable();

			if (!m_driveRotate->IsRotating()) {
				m_driveRotate->PIDDisable();
				AutonSteps++;
			}
			break;
		case 1:
			m_drvStraightPID->SetSetpoint(-64);
			m_drvStraightPID->Enable();

			if ((fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
					< 5) && (fabs(
					m_armEncoder->PIDGet() - m_armPID->GetSetpoint()) < 5)) {
				m_drvStraightPID->Disable();
				m_lEncode->Reset();
				m_rEncode->Reset();
				AutonSteps++;
			}
			break;
		case 2:
			m_ramCase = 0;
			if (m_ramCase == 3)
				AutonSteps++;
			break;
		case 3:
			m_armPID->SetSetpoint(0.0);

			if ((fabs(m_armEncoder->PIDGet() - m_armPID->GetSetpoint()) < 5)
					&& m_armPID->IsEnabled())
				m_armPID->Disable();

			break;
		}
	}

	void AutonTurnRight() {

		switch (AutonSteps) {
		case 0:
			m_driveRotate->SetAngle(0, 0, -18);
			m_driveRotate->PIDEnable();

			m_armPID->SetSetpoint(MED_SHOT_BACK);
			m_armPID->Enable();

			if (!m_driveRotate->IsRotating()) {
				m_driveRotate->PIDDisable();
				AutonSteps++;
			}
			break;
		case 1:
			m_drvStraightPID->SetSetpoint(-64);
			m_drvStraightPID->Enable();

			if ((fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint())
					< 5) && (fabs(
					m_armEncoder->PIDGet() - m_armPID->GetSetpoint()) < 5)) {
				m_drvStraightPID->Disable();
				m_lEncode->Reset();
				m_rEncode->Reset();
				AutonSteps++;
			}
			break;
		case 2:
			m_ramCase = 0;
			if (m_ramCase == 3)
				AutonSteps++;
			break;
		case 3:
			m_armPID->SetSetpoint(0.0);

			if ((fabs(m_armEncoder->PIDGet() - m_armPID->GetSetpoint()) < 5)
					&& m_armPID->IsEnabled())
				m_armPID->Disable();

			break;
		}
	}

	void AutonShotSafety() {
		if (!m_autonShot && m_ds->GetMatchTime() > 8.0) {
			m_ramCase = 0;
			m_autonShot = true;
		}
	}
	/*	
	 void AutonTracker () {
	 double ballX;
	 
	 switch (AutonSteps) {
	 case 0:		// Drive Foward + Arm Set Point
	 // Drive Foward
	 AutonStraightDrive(1,32 * REV_IN);
	 
	 // Arm Set Point
	 m_arm->SetAngle(MED_SHOOT_POS);
	 m_arm->PIDEnable();
	 
	 // Check if arm and drive are ready
	 if (Drive_Status) {
	 m_arm->PIDDisable();
	 AutonSteps++;
	 }
	 break;
	 case 1:		// Move to Center of the Hot Goal + Shoot
	 m_robotDrive->ArcadeDrive(0.0, m_cameraHandler->getCenter());
	 
	 if (fabs(m_cameraHandler->getCenter()) < ROTATE_ANGLE_GAP) {
	 m_ramCase = 0;
	 AutonSteps++;
	 }
	 break;
	 case 2:		// Dance and find Ball
	 // Dance
	 Dance();
	 
	 // Move The Arm to the Pick up position
	 m_arm->SetAngle(FLOOR_PICKING_POS);
	 m_arm->PIDEnable();
	 
	 // Check Ball
	 ballX = m_cameraHandler->getBallX();
	 if (-1.0 < ballX && ballX < 1.0) {
	 // Find The Ball
	 AutonSteps++;
	 }
	 break;
	 case 3:		// Track Ball
	 m_roller->Set(-1.0);
	 ballX = m_cameraHandler->getBallX();
	 if (-1.0 < ballX && ballX < 1.0) {
	 m_robotDrive->ArcadeDrive(0.8, ballX);
	 }
	 else if (-1.0 > ballX && ballX > 1.0){
	 m_autonTime->Start();
	 m_autonTime->Reset();
	 if (m_autonTime-> HasPeriodPassed(1.0)){
	 AutonSteps++;
	 }
	 else{
	 m_robotDrive->ArcadeDrive(1.0,0.0);
	 }
	 
	 }
	 
	 break;
	 case 4:
	 m_robotDrive->TankDrive(-.25,.25);
	 if (-1.0 < ballX && ballX < 1.0) {
	 AutonSteps = 3;
	 }
	 if (m_autonTime-> HasPeriodPassed(2.0)){
	 AutonSteps++;
	 }
	 break;
	 case 5:
	 m_autonTime->Stop();
	 m_arm->SetAngle(MED_SHOOT_POS);
	 m_arm->PIDEnable();
	 if (fabs(m_cameraHandler->getCenter()) < ROTATE_ANGLE_GAP) {
	 m_ramCase = 0;
	 AutonSteps++;
	 break;
	 case 6:
	 break;
	 }
	 }
	 }
	 
	 void Dance(){
	 switch(autondance){
	 case 0:     //
	 if (m_lEncode > m_rEncode){
	 m_driveRotate->SetAngle(61.87 - (.5 * CAMERA_VIEW));
	 m_driveRotate->PIDEnable();
	 if(!m_driveRotate->IsRotating()){
	 autondance = 1;
	 m_driveRotate->PIDDisable();
	 }
	 }
	 else if (m_rEncode > m_lEncode){
	 m_driveRotate->SetAngle(-61.87 + (.5 * CAMERA_VIEW));
	 m_driveRotate->PIDEnable();
	 if(!m_driveRotate->IsRotating()){
	 autondance = 2;
	 m_driveRotate->PIDDisable();
	 }
	 }
	 break;

	 case 1:
	 m_driveRotate->SetAngle(-180 + CAMERA_VIEW);
	 m_driveRotate->PIDEnable();
	 if(!m_driveRotate->IsRotating()){
	 autondance = 2;
	 m_driveRotate->PIDDisable();
	 }
	 break;

	 case 2:
	 m_driveRotate->SetAngle(180 - CAMERA_VIEW);
	 m_driveRotate->PIDEnable();
	 if(!m_driveRotate->IsRotating()){
	 autondance = 1;
	 m_driveRotate->PIDDisable();
	 }
	 break;
	 }
	 }
	 */

	/*********************** TELEOP FUNCTIONS **************************/
	double accelCap(double newValue) {
		double diff = newValue - m_currentVal;
		if (diff >= 0) {
			if (diff > ACCEL_CAP) {
				diff = ACCEL_CAP;
			}
		} else {
			if (diff < -ACCEL_CAP) {
				diff = -ACCEL_CAP;
			}
		}
		m_currentVal += diff;
		//SmartDashboard::PutNumber("accelCap diff: ", diff);
		//SmartDashboard::PutNumber("accelCap newValue: ", newValue);
		//SmartDashboard::PutNumber("accelCap m_currentVal: ", m_currentVal);
		//SmartDashboard::PutNumber("accelCap: ", ACCEL_CAP);
		return m_currentVal;
	}

	void TeleopDrive() {
		if (m_driver->GetRawButton(BUTTON_X)) {
			m_shifters->Set(true);
			m_autonDrive->Set(-64.0, 0.0);
			m_autonDrive->Enable();
		} else if (m_driver->GetRawButton(BUTTON_Y)) {
			m_shifters->Set(true);
			m_autonDrive->Set(0.0, 30.0);
			m_autonDrive->Enable();
		} else if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2 || fabs(m_driver->GetRawAxis(RIGHT_X)) > 0.2) {
			m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y), -m_driver->GetRawAxis(RIGHT_X));
			m_driveRotate->PIDDisable();
			// if (!m_driver->GetRawButton(BUTTON_START)) 
			//	m_driveRotate->PIDDisable();
		} else {
			m_robotDrive->ArcadeDrive(0.0, 0.0);
			m_autonDrive->Disable();
		}
		//Shifting
		if (m_shiftOverride == true) {
			m_shifters -> Set(true);
		} else if (m_shiftOverride == false) {
			m_shifters -> Set(false);
		}
		//Shift and Shift Override disable
		if (m_driver -> GetRawButton(BUTTON_LB)) {
			m_shifters -> Set(true);
			m_shiftOverride = false;
		}

		if (m_driver->GetRawButton(BUTTON_BACK)) {
			m_rEncode->Reset();
			m_lEncode->Reset();
		}

		/*
		 SmartDashboard::PutNumber("Auton Drive Rotate P: ", m_autonDrive->GetRotateP());
		 SmartDashboard::PutNumber("Auton Drive Rotate I: ", m_autonDrive->GetRotateI());
		 SmartDashboard::PutNumber("Auton Drive Rotate D: ", m_autonDrive->GetRotateD());
		 SmartDashboard::PutNumber("Auton Drive Straight P: ", m_autonDrive->GetStraightP());
		 SmartDashboard::PutNumber("Auton Drive Straight I: ", m_autonDrive->GetStraightI());
		 SmartDashboard::PutNumber("Auton Drive Straight D: ", m_autonDrive->GetStraightD());
		 if (m_driver->GetRawButton(BUTTON_A) && m_driver->GetRawButton(BUTTON_START))
		 m_autonDrive->SetStraightPID(m_autonDrive->GetStraightP() + 0.01, m_autonDrive->GetStraightI(), m_autonDrive->GetStraightD());
		 else if (m_driver->GetRawButton(BUTTON_B) && m_driver->GetRawButton(BUTTON_START))
		 m_autonDrive->SetStraightPID(m_autonDrive->GetStraightP() - 0.01, m_autonDrive->GetStraightI(), m_autonDrive->GetStraightD());
		 else if (m_driver->GetRawButton(BUTTON_A))
		 m_autonDrive->SetRotatePID(m_autonDrive->GetRotateP() + 0.01, m_autonDrive->GetRotateI(), m_autonDrive->GetRotateD());
		 else if (m_driver->GetRawButton(BUTTON_B))
		 m_autonDrive->SetRotatePID(m_autonDrive->GetRotateP() - 0.01, m_autonDrive->GetRotateI(), m_autonDrive->GetRotateD());
		 // else if (m_driver->GetRawButton(BUTTON_X))
		 // 	m_drvStraightPID->SetPID(m_drvStraightPID->GetP(),m_drvStraightPID->GetI(),(m_drvStraightPID->GetD()+0.01));
		 // else if (m_driver->GetRawButton(BUTTON_Y))
		 //	m_drvStraightPID->SetPID(m_drvStraightPID->GetP(),m_drvStraightPID->GetI(),(m_drvStraightPID->GetD()-0.01));
		 if (m_driver->GetRawButton(BUTTON_Y))
		 {
		 m_autonDrive->Set(24.0, 0.0);
		 }
		 else if (m_driver->GetRawButton(BUTTON_X))
		 {
		 m_autonDrive->Set(0.0, 30.0);
		 }
		 */
	}

	void TeleopBGrabber() {
		if (m_driver->GetRawButton(BUTTON_Y)) {
			m_rollTime->Stop();
			m_rollTime->Start();
			m_rollTime->Reset();
		}

		if (m_rollTime->HasPeriodPassed(0.5)) {
			m_dunRollin = true;
			m_rollTime->Stop();
		}

		//ROLLERS
		if (!m_dunRollin)
			m_roller->Set(-1);
		else if (m_operator->GetRawAxis(TRIGGERS) > 0.4
				|| m_driver->GetRawButton(BUTTON_RB)) {
			m_roller->Set(1);
		} else if (m_operator->GetRawAxis(TRIGGERS) < -0.4)
			m_roller->Set(-1);
		else if (fabs(m_armEncoder->GetRate()) > 250 && m_ramCase == -1) {
			m_roller->Set(-1);
			m_dunRollin = false;
			m_rollTime->Stop();
			m_rollTime->Start();
			m_rollTime->Reset();
		} else if (m_ramCase != 1 && m_ramCase != 0) {
			m_roller->Set(0.0);
		}

		//BALL CATCH
		if (m_operator->GetRawButton(BUTTON_RB)) {
			m_catch->Set(true);
		} else
			m_catch->Set(false);
		/*
		 //BALL CATCH (#Sweg)
		 if (m_operator->GetRawButton(BUTTON_BACK)) {
		 
		 m_catch->Set(true); 
		 }
		 else {
		 m_catch->Set(false);
		 }
		 */
		//BALL CATCH SERVOS
		/*
		 if (m_operator->GetRawButton(BUTTON_START))
		 {
		 m_catchServo1->SetAngle(180);
		 m_catchServo2->SetAngle(180);
		 }
		 else
		 {
		 m_catchServo1->SetAngle(30);
		 m_catchServo2->SetAngle(30);
		 }
		 */
		//bArm OPEN / CLOSE
		if (m_operator->GetRawButton(BUTTON_START))
			m_bArm->Set(true);
		else if (m_ramCase == -1)
			m_bArm->Set(false);

	}

	void TeleopArm() {
		// ----- PID -----
		if (m_operator->GetRawButton(BUTTON_BACK) && m_operator->GetRawButton(BUTTON_START))
			m_armEncoder->Reset();
		if (m_operator->GetRawButton(BUTTON_A)) {
			// Floor Picking
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(ApplyArmOffset(FLOOR_PICKING_POS));
			m_armPID->Enable();

		} else if (m_operator->GetRawButton(BUTTON_LB) && m_operator->GetRawButton(BUTTON_B)) {
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(ApplyArmOffset(GUARDED_SHOT_BACK));
			m_armPID->Enable();
		} else if (m_operator->GetRawButton(BUTTON_B)) {
			// Medium (12ft) Shoot Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(ApplyArmOffset(MED_SHOT_BACK));
			m_armPID->Enable();

		} else if (m_operator->GetRawButton(BUTTON_LB)
				&& m_operator->GetRawButton(BUTTON_X)) {
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(ApplyArmOffset(GUARDED_SHOT_FRONT));
			m_armPID->Enable();
		} else if (m_operator->GetRawButton(BUTTON_LB)
				&& m_operator->GetRawButton(BUTTON_Y)) {
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(ApplyArmOffset(GUARDED_SHOT));
			m_armPID->Enable();
		} else if (m_operator->GetRawButton(BUTTON_X)) {
			// Long (18ft) Shoot Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(ApplyArmOffset(MED_SHOOT_POS));
			m_armPID->Enable();

		} else if (m_operator->GetRawButton(BUTTON_Y)
				&& !m_operator->GetRawButton(BUTTON_BACK)) {
			// Catch Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(ApplyArmOffset(TRUSS_SHOT));
			m_armPID->Enable();

		} else {
			if (m_armPIDFlag) {
				m_armPID->Disable();
				m_armWrite->Reset();
				m_armPIDFlag = false;
			}

			// Control With Joystick
			if (!m_operator->GetRawButton(BUTTON_BACK))
				m_armMotor->Set(m_operator->GetRawAxis(LEFT_Y));

		}

		if (m_armPID->IsEnabled() && !m_operator->GetRawButton(BUTTON_B) && !m_operator->GetRawButton(BUTTON_A) && fabs(m_armEncoder->GetDistance() - m_armPID->GetSetpoint()) < 15) {
			m_armPID->SetPID(m_armPID->GetP(), 0.003, m_armPID->GetD());
		} else {
			m_armPID->SetPID(m_armPID->GetP(), ARM_I, m_armPID->GetD());
		}

		/*
		 if (m_driver->GetRawButton(BUTTON_A))
		 aCCEL_CAP +=0.1;
		 else if (m_driver->GetRawButton(BUTTON_B))
		 aCCEL_CAP -=0.11;*/
		//if (m_driver->GetRawButton(BUTTON_X))
		//	m_armWrite->ChangeMaxThrottle(0.001);
		//else if (m_driver->GetRawButton(BUTTON_Y))
		//	m_armWrite->ChangeMaxThrottle(-0.001);


		SmartDashboard::PutBoolean("Arm PID enabled ", m_armPID->IsEnabled());
		SmartDashboard::PutNumber("Arm Difference: ",
				fabs(m_armEncoder->GetDistance()) - m_armPID->GetSetpoint());

	}

	/*************************** TEST FUNCTIONS *****************************/

	void TestDrive() {

		if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2 || fabs(
				m_driver->GetRawAxis(RIGHT_X)) > 0.2)
			m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),
					-m_driver->GetRawAxis(RIGHT_X));
		else {
			if (m_driver->GetRawButton(BUTTON_START) && m_driver->GetRawButton(
					BUTTON_Y)) {
				if (!m_drvStraightPID->IsEnabled()) {
					m_drvStraightPID->SetSetpoint(32.0);
					m_drvStraightPID->Enable();
				}
			}

			else {
				m_driveRotate->PIDDisable();
				if (m_drvStraightPID->IsEnabled())
					m_drvStraightPID->Disable();
				m_robotDrive->ArcadeDrive(0.0, 0.0);
			}
		}

		/*
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"Right encoder count: %d",m_rEncode->Get());
		 m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"Left encoder count: %d",m_lEncode->Get());
		 */
	}

	void TestArm() {
		// Control Arm
		m_armMotor->Set(m_operator->GetRawAxis(LEFT_Y));

		// Reset Arm Encoder
		if (m_operator->GetRawButton(BUTTON_L3)) {
			m_armEncoder->Reset();
		}

	}

	void TestFindSensorWidth() {
		if (!m_armReset->Get() && !m_armResetFlag) {
			m_armResetPos = m_armEncoder->GetDistance();
			m_armResetFlag = true;
		}
		if (m_armReset->Get() && m_armResetFlag) {
			m_armResetPos -= m_armEncoder->GetDistance();
			m_armResetFlag = false;
		}

		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Reset Width: %f",
				(float) m_armResetPos);
		m_dsLCD->UpdateLCD();
	}

	void TestBGrabber() {
		//ROLLERS	
		if (m_operator->GetRawAxis(TRIGGERS) > 0.4) {
			m_roller->Set(1.0);
		} else if (m_operator->GetRawAxis(TRIGGERS) < -0.4)
			m_roller->Set(-1.0);
		else {
			m_roller->Set(0.0);
		}

		//BALL CATCH (#Sweg)
		if (m_operator->GetRawButton(BUTTON_A)) {
			m_catch->Set(true);
		} else if (m_operator->GetRawButton(BUTTON_B)) {
			m_catch->Set(false);
		}

		//bArm OPEN / CLOSE
		if (m_operator->GetRawButton(BUTTON_X)) {
			m_bArm->Set(true);
		} else if (m_operator->GetRawButton(BUTTON_Y)) {
			m_bArm->Set(false);
		}
	}

	void TestRamMotion() {
		if (m_driver->GetRawButton(BUTTON_LB))
			// IN
			m_ramMotor->Set(1);
		else if (m_driver->GetRawButton(BUTTON_RB))
			// OUT
			m_ramMotor->Set(-1);
		else
			m_ramMotor->Set(0);
	}

	void TestRamLock() {
		if (m_driver->GetRawAxis(TRIGGERS) < -.2) {
			if (m_driver->GetRawButton(BUTTON_BACK))
				m_ramServo->SetAngle(180);
			else
				m_ramServo->SetAngle(120);
		} else {
			m_ramServo->SetAngle(0);
		}

	}

	/************** UNIVERSAL FUNCTIONS ***************/

	void ManageCompressor() {
		if (m_compressor->GetPressureSwitchValue()) {
			m_compressor->Stop();
		} else {
			m_compressor->Start();
		}
	}

	void ArmReset() {
		double dirToZero;

		if (!m_armReset->Get() && !m_armResetFlag) {
			m_armResetPos = m_armEncoder->GetDistance();
			m_canResetArm = true;
			m_armResetFlag = true;
		} else
			m_canResetArm = false;

		// RESET VARIABLES WHEN OUTSIDE RANGE *****************************************

		if (m_armReset->Get()) {
			m_canResetArm = false;
			m_armResetFlag = false;
		}

		dirToZero = m_armEncoder->GetDistance() - m_armResetPos;

		SmartDashboard::PutNumber("Direction to Arm Zero: ", dirToZero);

		// RESET ARM ******************************************************************

		if ((m_operator->GetRawButton(BUTTON_BACK) && m_operator->GetRawButton(
				BUTTON_Y)) || AutonDBSteps == 7) {
			if (m_canResetArm) {
				//Moving Towards Front
				if (m_armEncoder->GetRate() > 0)
					m_armOffset = m_armEncoder->GetDistance() + ARM_RESET_WIDTH;

				//Moving Towards Back
				else if (m_armEncoder->GetRate() < 0)
					m_armOffset = m_armEncoder->GetDistance();

				m_armMotor->Set(0.0);
				m_armPID->Reset();
				m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
						"                     ");
				m_armResetStop = true;
			} else {
				if (m_armResetStop)
					m_armMotor->Set(0.0);
				else if (dirToZero < 0)
					m_armMotor->Set(0.2);
				else if (dirToZero > 0)
					m_armMotor->Set(-0.2);
			}
		}

		// PRINT MESSAGE **************************************************************

		else {
			if (m_canResetArm && ((fabs(
					ApplyArmOffset(m_armEncoder->GetDistance())) > 10)
					|| (fabs(
							ApplyArmOffset(m_armEncoder->GetDistance())
									- ARM_RESET_WIDTH) > 10)))
				m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
						"RESET ARM NOW PLEASE ");

			m_armResetStop = false;
		}

		m_dsLCD->UpdateLCD();
	}

	void AutoArmReset() {
		if (!m_armReset->Get() && !m_armResetFlag) {
			m_armResetPos = m_armEncoder->GetDistance();
			m_canResetArm = true;
			m_armResetFlag = true;
		} else
			m_canResetArm = false;

		// RESET VARIABLES WHEN OUTSIDE RANGE *****************************************

		if (m_armReset->Get()) {
			m_canResetArm = false;
			m_armResetFlag = false;
		}

		// RESET ARM ******************************************************************
		if (m_canResetArm) {
			//Moving Towards Front
			/*
			 if (m_armEncoder->GetRate() > 0)
			 m_armOffset = m_armEncoder->GetDistance() + ARM_RESET_WIDTH;
			 */
			//Moving Towards Back
			/*else*/
			if (m_armEncoder->GetRate() < 0)
				m_armOffset = m_armEncoder->GetDistance();

			m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"                     ");
			m_armResetStop = true;
		}

		// PRINT MESSAGE **************************************************************

		else {
			m_armResetStop = false;
		}

		m_dsLCD->UpdateLCD();
	}

	void WatchArm() {
		if (fabs(ApplyArmOffset(MED_SHOT_BACK) - m_armEncoder->GetDistance()) < 100 && (m_operator->GetRawButton(BUTTON_B) || m_ds->IsAutonomous())) {
			m_armPID->SetPID((ARM_P - .005) * (fabs((ApplyArmOffset(MED_SHOT_BACK) - m_armEncoder->GetDistance()) / 100)) + .005, 0, 0);
		} else if (fabs(ApplyArmOffset(FLOOR_PICKING_POS) - m_armEncoder->GetDistance()) < 100 && (m_operator->GetRawButton(BUTTON_A))) {
			m_armPID->SetPID((ARM_P - .005) * (fabs((ApplyArmOffset(FLOOR_PICKING_POS) - m_armEncoder->GetDistance()) / 100)) + .005, 0, 0); 
		} else {
			if (m_armPID->GetP() != ARM_P)
				m_armPID->SetPID(ARM_P, m_armPID->GetI(), ARM_D);
		}
	}

	double ApplyArmOffset(double input) {
		return input + m_armOffset;
	}

	void RamrodInit() {
		if (!m_ramInit) {
			m_ramServo->SetAngle(0);
			m_ramMotor->Set(-0.15);
			if (m_ramTime->HasPeriodPassed(0.3)) {
				if (abs(m_ramEncode->GetRate()) < 5) {
					m_ramEncode->Reset();
					m_ramMotor->Set(0.0);
					m_ramTime->Stop();
					m_ramTime->Reset();
					m_ramInit = true;
					m_ramCase = -1;
				}

			}
		}
	}

	void AdvancedDrive() {
		if (m_driver->GetRawButton(BUTTON_LB)) {
			m_shifters->Set(true);
			m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),
					-m_driver->GetRawAxis(RIGHT_X));
		} else if (m_shifters->Get()) {
			if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2) {
				m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),
						-m_driver->GetRawAxis(RIGHT_X));
			} else if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.75) {
				m_shifters->Set(false);
			}
		} else if (!m_shifters->Get()) {
			if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.65) {
				m_robotDrive->ArcadeDrive(
						(2.4 * (-m_driver->GetRawAxis(LEFT_Y) - 75)) + .40,
						-m_driver->GetRawAxis(RIGHT_X));
			} else {
				m_shifters->Set(true);
			}
		}
	}

	void AutoDownShift() {
		/*
		 if((fabs(m_driver->GetRawAxis(LEFT_Y)) > .75) && fabs(((m_lEncode->GetRate()+m_rEncode->GetRate()) < 1000)))
		 {
		 if(m_currentTimer->Get() == 0.0)
		 {
		 m_currentTimer->Start();
		 m_currentTimer->Reset();
		 }
		 if (m_currentTimer->HasPeriodPassed(.5))
		 {
		 m_shiftOverride = true;
		 }
		 }
		 else if(m_currentTimer->HasPeriodPassed(.75))
		 {
		 m_currentTimer->Stop();
		 m_currentTimer->Reset();
		 }*/

		if (m_currentSensor->GetAverageVoltage() * 100 > 200) {
			if (m_currentTimer->Get() == 0.0) {
				m_currentTimer->Start();
				m_currentTimer->Reset();
			}
			if (m_currentTimer->HasPeriodPassed(1.0)) {
				m_shiftOverride = true;
			}
		} else {
			m_currentTimer->Stop();
			m_currentTimer->Reset();
		}

	}

	void RamFire() {
		//Do the thing with bGrabber
		if (m_driver->GetRawAxis(TRIGGERS) < -0.4 && m_ramCase == -1) {
			m_ramCase = 0;
		}
		/*else if (m_driver->GetRawAxis(TRIGGERS) > 0.4 && m_ramCase == -1)
		 {
		 m_ramCase = 3;
		 }*/

		switch (m_ramCase) {
		case 0:
			m_ramTime->Stop();
			m_ramTime->Start();
			m_ramTime->Reset();

			m_roller->Set(-1.0);

			m_bArm->Set(true);

			m_ramCase++;
			break;
		case 1:
			m_ramServo->SetAngle(180);
			if (m_ramTime->Get() > 0.15) {
				m_roller->Set(0.0);
			}
			if (m_ramTime->HasPeriodPassed(1.0))
				m_ramCase++;
			break;
		case 2:
			m_ramServo->SetAngle(0);
			m_ramCase++;
			m_ramTime->Stop();
			m_ramTime->Start();
			m_ramTime->Reset();
			m_ramMotor->Set(0.8);
			break;
		case 3:
			if (m_ramTime->HasPeriodPassed(0.1)) {
				if (abs(m_ramEncode->GetRate()) < 20) {
					m_ramMotor->Set(0.0);
					m_ramTime->Stop();
					m_ramTime->Start();
					m_ramTime->Reset();
					m_ramCase++;
				}
			}
			break;
		case 4:
			if (m_ramTime->HasPeriodPassed(0.1)) {
				if (abs(m_ramEncode->GetDistance()) < 200) {
					m_ramMotor->Set(-0.15);
					m_bArm->Set(false);
					m_ramCase++;
				} else
					m_ramMotor->Set(-0.4);
			}
			break;
		case 5:
			if (abs(m_ramEncode->GetRate()) < 5) {
				m_ramEncode->Reset();
				m_ramMotor->Set(0.0);
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();
				m_ramCase = -1;
			}
			break;
		}
	}
	/*
	 switch(m_ramCase)
	 {
	 case 0:
	 m_ramTime->Stop();
	 m_ramTime->Start();
	 m_ramTime->Reset();
	 m_ramCase++;
	 break;
	 case 1:
	 m_ramServo->SetAngle(180);
	 if(m_ramTime->HasPeriodPassed(0.7))
	 m_ramCase++;
	 break;
	 case 2:
	 m_ramServo->SetAngle(30);
	 m_ramCase++;
	 m_ramTime->Stop();
	 m_ramTime->Start();
	 m_ramTime->Reset();			
	 break;
	 case 3:
	 if(m_ramTime->HasPeriodPassed(0.3))
	 {
	 m_ramTime->Stop();
	 m_ramTime->Start();
	 m_ramTime->Reset();	
	 m_ramCase++;
	 }
	 else
	 m_ramMotor->Set(1);
	 break;
	 case 4:
	 if (m_ramTime->HasPeriodPassed(0.5))
	 {
	 m_ramTime->Stop();
	 m_ramTime->Start();
	 m_ramTime->Reset();
	 m_ramCase++;
	 }
	 else
	 m_ramMotor->Set(0.7);
	 break;
	 case 5:
	 if (m_ramTime->HasPeriodPassed(0.15))
	 {
	 m_ramTime->Stop();
	 m_ramTime->Start();
	 m_ramTime->Reset();
	 m_ramCase++;
	 }
	 else
	 m_ramMotor->Set(-1);
	 break;
	 case 6:
	 if (m_ramTime->HasPeriodPassed(0.3))
	 {
	 m_ramTime->Stop();
	 m_ramTime->Start();
	 m_ramTime->Reset();
	 m_ramCase++;
	 }
	 else
	 m_ramMotor->Set(-0.2);
	 break;
	 case 7:
	 m_ramMotor->Set(0.0);
	 m_ramTime->Stop();
	 m_ramCase = -1;
	 break;
	 }
	 }
	 */
	void MedRamFire() {
		if (m_driver->GetRawAxis(TRIGGERS) > 0.4 && m_ramCase == -1
				&& m_medRamCase == -1)
			m_medRamCase = 0;
		switch (m_medRamCase) {
		case 0:
			if (abs(m_ramEncode->GetDistance()) > (RAM_MID_POSITION - 100))
				m_ramMotor->Set(1);
			else
				m_medRamCase++;
			break;
		case 1:
			if (abs(m_ramEncode->GetDistance()) > (RAM_MID_POSITION - 20)) {
				m_ramMotor->Set(0.2);
				m_medRamCase++;
			} else
				m_ramMotor->Set(1.0);
			break;
		case 2:
			if (m_ramEncode->GetDistance() > (RAM_MID_POSITION - 50)) {
				m_ramMotor->Set(0.0);
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();
				m_medRamCase++;
			}
			break;
		case 3:
			if (m_ramServo->GetAngle() != 120)
				m_ramServo->SetAngle(120);
			if (m_ramTime->HasPeriodPassed(0.3)) {
				m_ramTime->Stop();
				m_ramTime->Reset();
				m_medRamCase++;
			}
			break;
		case 4:
			if (m_ramEncode->GetDistance() > 20) {
				m_ramMotor->Set(-0.3);
			} else {
				m_ramCase = 2;
				m_medRamCase = -1;
			}
			break;
		}
	}

	void RamrodOverride() {
		if (m_operator->GetRawButton(BUTTON_BACK)) {
			SmartDashboard::PutNumber("Joystick Value: ",
					m_operator->GetRawAxis(RIGHT_Y));

			if (fabs(m_operator->GetRawAxis(RIGHT_Y)) > 0.2)
				m_ramMotor->Set(m_operator->GetRawAxis(RIGHT_Y));
			else
				m_ramMotor->Set(0.0);
		}
	}

	void PrintData() {
		if (!m_ds->IsFMSAttached() || m_printDataInMatch) {
			SmartDashboard::PutNumber("Ramrod Encoder: ",
					m_ramEncode->GetDistance());
			/*
			 SmartDashboard::PutNumber("Ramrod Rate: ",m_ramEncode->GetRate());
			 SmartDashboard::PutNumber("Ramrod Raw: ",m_ramEncode->GetRaw());
			 */
			SmartDashboard::PutNumber("Ramrod Case: ", m_ramCase);
			//SmartDashboard::PutNumber("Ram Time: ",m_ramTime->Get());
			SmartDashboard::PutNumber("Medium Ramrod Fire Case: ", m_medRamCase);

			SmartDashboard::PutNumber("Arm Calc Position Offset: ",
					m_armEncoder->GetDistance() - m_armOffset);
			SmartDashboard::PutNumber("Arm Actual Position: ",
					m_armEncoder->GetDistance());

			/*
			 SmartDashboard::PutNumber("Arm Rate: ", m_armEncoder->GetRate());
			 
			 
			 SmartDashboard::PutNumber("Arm PID Input: ", m_armEncoder->PIDGet());
			 
			 SmartDashboard::PutNumber("Arm Motor Input:",m_operator->GetRawAxis(LEFT_Y));
			 SmartDashboard::PutBoolean("Arm Difference Bool", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS) < AUTON_ANGLE_GAP);
			 */

			SmartDashboard::PutNumber("lEncoder: ", m_rEncode->GetDistance());
			SmartDashboard::PutNumber("rEncoder: ", m_lEncode->GetDistance());
			SmartDashboard::PutNumber("Encoders Added:",
					(m_lEncode->GetRate() + m_rEncode->GetRate()));

			SmartDashboard::PutNumber("Arm Reset Position: ", m_armResetPos);
			SmartDashboard::PutNumber("Arm Offset: ", m_armOffset);
			SmartDashboard::PutBoolean("Arm Reset Sensor", m_armReset->Get());
			SmartDashboard::PutBoolean("Can reset arm? ", m_canResetArm);
			SmartDashboard::PutBoolean("Arm reset flag: ", m_armResetFlag);

			SmartDashboard::PutNumber("Max Arm Throttle: ",
					m_armWrite->GetMaxThrottle());
			SmartDashboard::PutNumber("Current Arm Throttle: ",
					m_armMotor->Get());
			SmartDashboard::PutNumber("Last Arm Throttle: ",
					m_armWrite->GetLastValue());
			SmartDashboard::PutNumber("Arm PID Output:", m_armPID->Get());

			SmartDashboard::PutBoolean("dunRollin: ", m_dunRollin);
			SmartDashboard::PutNumber("Roll Time: ", m_rollTime->Get());

			SmartDashboard::PutNumber("Arm P: ", m_armPID->GetP());
			SmartDashboard::PutNumber("Arm I: ", m_armPID->GetI());
			SmartDashboard::PutNumber("Arm D: ", m_armPID->GetD());
			SmartDashboard::PutNumber("D Offset: ", m_armDOffset);

			// Auton
			SmartDashboard::PutNumber("2-Ball Auton Step: ", AutonDBSteps);
			SmartDashboard::PutNumber("Auton Step: ", AutonSteps);
			SmartDashboard::PutBoolean("Auton Time: ", m_autonTime->Get());
			SmartDashboard::PutNumber("Driver Y Axis: ",
					fabs(m_driver->GetRawAxis(LEFT_Y)));
			SmartDashboard::PutNumber("Current Timer: ", m_currentTimer->Get());
			SmartDashboard::PutNumber("Current Sensor (A): ",
					m_currentSensor->GetAverageVoltage() * 100);
			SmartDashboard::PutBoolean("Shift Override: ", m_shiftOverride);

			//Current sensor
			SmartDashboard::PutNumber("Current: ",
					m_currentSensor->GetVoltage() * 100);
			SmartDashboard::PutNumber("Current(Average): ",
					m_currentSensor->GetAverageVoltage() * 100);

			// CheesyVisionServer
			SmartDashboard::PutBoolean("Left Hot: ",
					m_cheesyVisionServer->GetLeftStatus());
			SmartDashboard::PutNumber("Left Count: ",
					m_cheesyVisionServer->GetLeftCount());
			SmartDashboard::PutBoolean("Right Hot: ",
					m_cheesyVisionServer->GetRightStatus());
			SmartDashboard::PutNumber("Right Count: ",
					m_cheesyVisionServer->GetRightCount());
			SmartDashboard::PutBoolean("Has Client: ",
					m_cheesyVisionServer->HasClientConnection());
			/*
			 SmartDashboard::PutNumber("Drive PID Output: ",m_drvStraightPID->Get());
			 SmartDashboard::PutNumber("Drive PID Input: ",(m_lEncode->GetDistance()+m_rEncode->GetDistance())/(2.0*REV_IN));
			 SmartDashboard::PutNumber("Left Drive Set: ",m_lDrive->Get());
			 SmartDashboard::PutNumber("Right Drive Set: ",m_rDrive->Get());
			 SmartDashboard::PutNumber("Drive P: ",m_drvStraightPID->GetP());
			 SmartDashboard::PutNumber("Drive D: ",m_drvStraightPID->GetD());
			 SmartDashboard::PutBoolean("Left Hot: ", m_cheesyVisionServer->GetLeftStatus());
			 SmartDashboard::PutNumber("Left Count: ", m_cheesyVisionServer->GetLeftCount());
			 SmartDashboard::PutBoolean("Right Hot: ", m_cheesyVisionServer->GetRightStatus());
			 SmartDashboard::PutNumber("Right Count: ", m_cheesyVisionServer->GetLeftCount());
			 */
		}
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode)
;
