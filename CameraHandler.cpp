#include "cameraHandler.h"
#define TWO_IMAGES
	CameraHandler::CameraHandler(AxisCamera *camera, DriverStationLCD *m_dsLCD, Relay *relay)
	{
		//This runs once when cameraHandler Class is initilized
		camera->WriteResolution(AxisCamera::kResolution_320x240);
		camera->WriteBrightness(40);

		this->img = new ColorImage(IMAQ_IMAGE_HSL);
		//this->img2 = new ColorImage(IMAQ_IMAGE_HSL);
		this->camera = camera;
		this->m_dsLCD = m_dsLCD;
		this->light = relay;
		this->m_ds = DriverStation::GetInstance();
	}

	bool particleSort (ParticleAnalysisReport i, ParticleAnalysisReport j) {return (i.particleArea > j.particleArea);}


	double CameraHandler::getCenter()
	{
		unsigned x,y;

		int largestWidth;
		int largestHeight[2];
		int largestWidthVal, largestHeightVal;

		BinaryImage* binImg;
		vector<ParticleAnalysisReport>* particles;

		//m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Test");
		//m_dsLCD->UpdateLCD();
		//Get new camera image
		camera->GetImage(img);

		//img.Write("bob2.jpg");  //Cannot work with non-RGB images

		//int Wid = img->GetWidth();  //Sanity Check: Check width of image
		//Prints width of image from camera
		//m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,"Width of Image: %d",Wid);

		//Perform HSLThreshold to pull out only blue LED reflection of targets into BinaryImage
		//BinaryImage* binImg = img->ThresholdHSL(202, 255, 55, 255, 55, 129);      //RED LED WORKS TERRRIBLY!!!!
		binImg = img->ThresholdHSL(52, 255, 71, 188, 76, 219);      //RED LED WORKS TERRRIBLY!!!!
		//BinaryImage* binImg = img->ThresholdHSL(57, 255, 79, 255, 51, 255);  //BLUE LED
		//BinaryImage* binImg = img->ThresholdHSL(159, 255, 0, 255, 71, 255);  //RED LED
		//Perform Morphology on image to remove noise/unwanted particles.  Also fills in incomplete particles.
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_PCLOSE);

		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_DILATE);
		//Perform particle analysis on BinaryImage, creates vector with all particles found
		particles = binImg->GetOrderedParticleAnalysisReports();
		
		printf("Particles found: %d",(int)particles->size());

		//Print numbers of particles found to driver station
		//m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "# Parts:%d    ",particles->size());
		//m_dsLCD->UpdateLCD();
		if(particles->size() > 1 || particles->size() < 30)
		{// Sort by size
			sort(particles->begin(), particles->end(), particleSort);

			// Initialize
			y=0;

			largestWidth = 0;
			largestHeight[0] = -1;
			largestHeight[1] = -1;
			largestHeightVal = 0;
			largestWidthVal = 0;
			largestHeightVal = 10;

			for (x=0; (x < 3 &&  x < particles->size()); x++) {
				// Find tallest
				if ((*particles)[x].boundingRect.height > largestHeightVal) {
					largestHeight[y] = x;
					largestHeightVal = (*particles)[x].boundingRect.height;
					y++;
				}

				// Find Fattest
				if ((*particles)[x].boundingRect.width > largestWidthVal) {
					largestWidth = x;
					largestWidthVal = (*particles)[x].boundingRect.width;
				}
			}

			// Detect Which Is Hot And Return Normalized Value of X (-1.0 - 1.0)
			if (fabs((*particles)[largestHeight[0]].center_mass_x - (*particles)[largestWidth].center_mass_x) < fabs((*particles)[largestHeight[1]].center_mass_x - (*particles)[largestWidth].center_mass_x)) {
				return (*particles)[largestHeight[0]].center_mass_x_normalized;
			} else {
				return (*particles)[largestHeight[1]].center_mass_x_normalized;
			}
		}
		else{
			return 0;
		}
	}

	state_t CameraHandler::getHotGoal ()
	{
		unsigned x;

		int largestWidth;		// Index of Particle
		int largestHeight;		// Index of Particle
		int largestWidthVal;	// Actual Width
		int largestHeightVal;	// Actual Height

		BinaryImage* binImg;
		vector<ParticleAnalysisReport>* particles;

		// Get Camera Image
		camera->GetImage(img);

		// Filter out Background
		binImg = img->ThresholdHSL(52, 255, 71, 188, 76, 219);

		// Make picture clear
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_PCLOSE);
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_DILATE);

		// Get Particle Analysis
		particles = binImg->GetOrderedParticleAnalysisReports();

		if (particles->size() == 1) {
			// Find Only One Particle
			return kNone;
		} else if (particles->size() > 0 && particles->size() < 30) {
			// Sort by size
			sort(particles->begin(), particles->end(), particleSort);

			// Initialize
			largestWidth = 0;
			largestHeight = 0;
			largestWidthVal = 0;
			largestHeightVal = 0;

			for (x=0; (x < 3 &&  x < particles->size()); x++) {
				// Find tallest
				if ((*particles)[x].boundingRect.height > largestHeightVal) {
					largestHeight = x;
					largestHeightVal = (*particles)[x].boundingRect.height;
				}

				// Find Fattest
				if ((*particles)[x].boundingRect.width > largestWidthVal) {
					largestWidth = x;
					largestWidthVal = (*particles)[x].boundingRect.width;
				}
			}

			if ((*particles)[largestWidth].center_mass_x < (*particles)[largestHeight].center_mass_x) {
				return kLeft;
			}
			else if ((*particles)[largestWidth].center_mass_x > (*particles)[largestHeight].center_mass_x) {
				return kRight;
			}
			else {
				return kNone;
			}

		} else {
			// Find Too Many Particles or None
			return kError;
		}
	}

	bool CameraHandler::getLeftHot ()
	{
		return getHotGoal() == kLeft;
	}

	bool CameraHandler::getRightHot ()
	{
		return getHotGoal() == kRight;
	}

	
	double CameraHandler::getBallX ()
	{
		unsigned x;

		int ballNum;
		double largestArea;
		double sizeRatio;

		BinaryImage* binImg;
		vector<ParticleAnalysisReport>* particles;

		// ----- Get Image -----
		camera->GetImage(img);

		// ----- Filter out background -----
		if (m_ds->GetAlliance() == DriverStation::kBlue)
					binImg = img->ThresholdHSV(160, 184, 120, 255, 14, 233);
				else
					binImg = img->ThresholdRGB(88, 255, 0, 74, 0, 31);

		// Make picture clear
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_PCLOSE);
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_ERODE);

		particles = binImg->GetOrderedParticleAnalysisReports();

		SmartDashboard::PutNumber("DEBUG Particle size: ", particles->size());
		
		if (particles->size() > 0 && particles->size() < 30)
		{
			sort(particles->begin(),particles->end(),particleSort);

			//Find ball

			largestArea = 25.0;
			ballNum = -1;
			
			for (x = 0; ((x < particles->size()) && x < 5); x++)
			{
				sizeRatio = (double)(*particles)[x].boundingRect.height/(*particles)[x].boundingRect.width;
				
				if (((*particles)[x].particleArea > largestArea) && (sizeRatio > 0.75 && sizeRatio < 1.25))
				{
					largestArea = (*particles)[x].particleArea;
					ballNum = x;
				}
			}
			
			if (ballNum == -1)
			{
				return -2.0;
			}
			else
			{
				return (*particles)[ballNum].center_mass_x_normalized;
			}
		}
		else {
			return -3.0;
		}

	}
  
	double CameraHandler::GetDistanceToBall ()
    {
		unsigned x;

		int ballNum;
		double objAngle;
		double largestArea;
		double sizeRatio;

		BinaryImage* binImg;
		vector<ParticleAnalysisReport>* particles;

		// ----- Get Image -----
		camera->GetImage(img);

		// ----- Filter out background -----
		if (m_ds->GetAlliance() == DriverStation::kBlue)
					binImg = img->ThresholdHSV(160, 184, 120, 255, 14, 233);
				else
					binImg = img->ThresholdRGB(88, 255, 0, 74, 0, 31);

		// Make picture clear
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_PCLOSE);
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_ERODE);

		particles = binImg->GetOrderedParticleAnalysisReports();
		
		SmartDashboard::PutNumber("DEBUG Particle size: ", particles->size());

		if (particles->size() > 0 && particles->size() < 30)
		{
			sort(particles->begin(),particles->end(),particleSort);

			//Find ball

			largestArea = 25.0;
			ballNum = -1;

			for (x = 0; ((x < particles->size()) && x < 5); x++)
			{
				sizeRatio = (double)(*particles)[x].boundingRect.height/(*particles)[x].boundingRect.width;
				
				if (((*particles)[x].particleArea > largestArea) && (sizeRatio > 0.75 && sizeRatio < 1.25))
				{
					largestArea = (*particles)[x].particleArea;
					ballNum = x;
				}
			}
		}

		if (ballNum >= 0)
		{
            objAngle = 0.5*((*particles)[ballNum].boundingRect.width)*(CAMERA_ANGLE/(*particles)[ballNum].imageWidth);

            return 1/tan(objAngle);
		}
		else
			return -1.0;
    }


