#include "main_LaneDetectorSim.h"
#include "Process_LaneDetectorSim.h"

//extern "C" {
	//#include "../LaneDetector/init.hh"
//}

//extern "C" {
#include "../LaneDetector/CameraInfoOpt.h"
//}

//extern "C" {
#include "../LaneDetector/LaneDetectorOpt.h"
//}

//extern "C" {
#include <stdexcept>
//}

//#include "cmdline.h"


#ifdef __cplusplus

/* Time */
extern const int    NUM_WINDOW_EWM      = 5;    //EWMA, EWVAR Init (times)
/* Size of Image */
extern const double COEF                = 1;
/* Multi-Image Show */
extern const int    WIN_COLS            = 3;
extern const int    WIN_ROWS            = 3;

/* Run applicaiton */
extern const int    IMAGE_RECORD        = 1;
/* Record docs */
extern const char   LANE_RECORD_FILE[]  = "./inputdata/outputdata/LaneFeatures_22-03-2014_13h05m12s.txt";
extern const char   FILE_LANE_FEATURE[] = "./inputdata/outputdata/Sim_LaneFeatures_22-03-2014_13h05m12s.txt";
extern const char   LANE_RECORD_IMAGE[]    = "./inputdata/outputdata/lane_%d.png";
/* Data Source */


//extern const char   LANE_RAW_NAME[]     = "./inputdata/cropped_images/cropped_%d.png";
extern const char   LANE_RAW_NAME[]     = "./inputdata/clips/lane_%d.png";
//extern const char   LANE_RAW_NAME[]     = "./inputdata/washington/lane_%d.png";
// extern const char   LANE_RAW_NAME[]     = "/home/lixp/Developer/KIT/2011_09_26/2011_09_26_drive_0015_sync/image_00/data/%010d.png";
// extern const char   LANE_RAW_NAME[]     = "/home/lixp/Developer/Data/LaneRaw_10-07-2013_18h30m21s/lane_%d.jpg";





extern const int    TH_KALMANFILTER     = 1; // originally 1

namespace LaneDetectorSim {

	int Process(int argc, const char* argv[])
	{
        	if(argc < 7)
            	std::cout << "Not enough parameters" << std::endl;

		int	LANE_DETECTOR  	= atoi(argv[1]);
		int	StartFrame    	= atoi(argv[2]); // FRAME_START
		int EndFrame	    	= atoi(argv[3]); // FRAME_END
    double YAW_ANGLE    = atof(argv[4]); // yaw - X
    double PITCH_ANGLE  = atof(argv[5]); // pitch - Y
		int	IPM_HK     	= atoi(argv[6]); //enables the IPM tranformation
	//	int	HK 	= atoi(argv[7]);
	  double coef_thetaMax = atof(argv[7]);

		std::cout << "/*************************************/" << std::endl;
		std::cout << "Input LANE_DETECTOR" << LANE_DETECTOR << std::endl;
		std::cout << "Input StartFrame" << StartFrame << std::endl;
		std::cout << "Input EndFrame" << EndFrame << std::endl;
		std::cout << "Input YAW_ANGLE" << YAW_ANGLE << std::endl;
		std::cout << "Input PITCH_ANGLE" << PITCH_ANGLE << std::endl;
		std::cout << "Input IPM_HK  " << IPM_HK  << std::endl; //0=IPM+HK, 1=IPM+P, 2=HK
	//	std::cout << "Input HK" << HK << std::endl;
		std::cout << "Input coef_thetaMax " << coef_thetaMax << std::endl;
		std::cout << "/*************************************/" << std::endl;


		int  idx            = StartFrame;  //index for image sequence
		int  sampleIdx      = 1;    //init sampling index
		char laneImg[100];

		//double initTime         = (double)cv::getTickCount();
		double intervalTime     = 0;
		double execTime         = 0;  // Execute Time for Each Frame
		double pastTime         = 0;
		double lastStartTime    = (double)cv::getTickCount();
		char key;
		double delay = 1;

		std::ofstream laneFeatureFile;

		/* Parameters for Lane Detector */
		cv::Mat laneMat, IPM_OUT;
		cv::Mat IPM_cont, particle_detect, particle_track;
		LaneDetector::LaneDetectorConf laneDetectorConf;
		std::vector<cv::Vec2f> hfLanes;
		std::vector<cv::Vec2f> lastHfLanes;
		std::vector<cv::Vec2f> preHfLanes;

		/*************JOOST*****************/

		LaneDetector_J::CameraInfo cameraInfo;
		char  fileName_test2[200];
		strcpy(fileName_test2, "./LaneDetector/CameraInfo3.conf");


		// read the configurations
		  LaneDetector_J::LaneDetectorConf_J lanesConf;
			char fileName_test[200];
			strcpy(fileName_test, "./LaneDetector/Lanes3.conf");


		/**************************************/

		std::vector<double> LATSDBaselineVec;
		std::deque<LaneDetector::InfoCar> lateralOffsetDeque;
		std::deque<LaneDetector::InfoCar> LANEXDeque;
		std::deque<LaneDetector::InfoTLC> TLCDeque;
		LaneDetector::LaneFeature laneFeatures;
		double lastLateralOffset = 0;
		double lateralOffset     = 0;    // Lateral Offset
		int    detectLaneFlag    = -1;   // init state -> normal state 0
		int    isChangeLane      = 0;    // Whether lane change happens
		int    changeDone        = 0;    // Finish lane change
		int    muWindowSize      = 5;    // Initial window size: 5 (sample)
		int    sigmaWindowSize   = 5;    // Initial window size: 5 (sample)
		std::vector<float> samplingTime;

		/* Initialize Lane Kalman Filter */
		cv::KalmanFilter laneKalmanFilter(8, 8, 0);//(rho, theta, delta_rho, delta_theta)x2
		cv::Mat laneKalmanMeasureMat(8, 1, CV_32F, cv::Scalar::all(0));//(rho, theta, delta_rho, delta_theta)
		int    laneKalmanIdx     = 0;    //Marker of start kalmam

		InitlaneFeatures(laneFeatures);
 if (LANE_DETECTOR){
		switch (IPM_HK) {

			case 0: //IPM+HK

			sprintf(laneImg, LANE_RAW_NAME , idx);
			laneMat = cv::imread(laneImg);
			LaneDetector_J::mcvInitLaneDetectorConf(fileName_test, &lanesConf);
		//	MSG("Loaded lanes config file\n");
			LaneDetector_J::mcvInitCameraInfo(fileName_test2, &cameraInfo);
		//	MSG("Loaded camera file\n");
			LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2, coef_thetaMax); // KIT 1, ESIEE 2
			LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);

			break;


			case 1:		//IPM+P

			sprintf(laneImg, LANE_RAW_NAME , idx);
			laneMat = cv::imread(laneImg);
			LaneDetector_J::mcvInitLaneDetectorConf(fileName_test, &lanesConf);
		//	MSG("Loaded lanes config file\n");
			LaneDetector_J::mcvInitCameraInfo(fileName_test2, &cameraInfo);
		//	MSG("Loaded camera file\n");
			LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2, coef_thetaMax); // KIT 1, ESIEE 2
			LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
			break;



			case 2:

			sprintf(laneImg, LANE_RAW_NAME , idx);
			laneMat = cv::imread(laneImg);
			LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2, coef_thetaMax); // KIT 1, ESIEE 2
			LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);

			break;
		}

	}
					//
        	// if (LANE_DETECTOR && IPM_HK)
	      	// {
          //    		/* Lane detect and tracking */
          //   		sprintf(laneImg, LANE_RAW_NAME , idx);
          //   		laneMat = cv::imread(laneImg);
					// 			LaneDetector_J::mcvInitLaneDetectorConf(fileName_test, &lanesConf);
					// 		//	MSG("Loaded lanes config file\n");
					// 			LaneDetector_J::mcvInitCameraInfo(fileName_test2, &cameraInfo);
					// 		//	MSG("Loaded camera file\n");
          //   		LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2, coef_thetaMax); // KIT 1, ESIEE 2
          //   		LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
					//
        	// }
					//
					// if (LANE_DETECTOR && HK)
	      	// {
          //    		/* Lane detect and tracking */
	        //     sprintf(laneImg, LANE_RAW_NAME , idx);
	        //     laneMat = cv::imread(laneImg);
	        //     LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2, coef_thetaMax); // KIT 1, ESIEE 2
	        //     LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
					//
        	// }



        	/* Inter-process communication */
        	key_t ipckey;
        	int mq_id;
        	struct
	      	{
            		long type;
            		char text[1024];
        	} laneMsg;

		double delayTime;
		if(idx == 1)
		delayTime = 0;
		else
		delayTime = samplingTime.at(idx - 2);

        	/* Entrance of Process */
        	while (idx <= EndFrame)
        	{
            		double startTime = (double)cv::getTickCount();

            		/* Lane detect and tracking */
            		sprintf(laneImg, LANE_RAW_NAME , idx);
            		laneMat = cv::imread(laneImg);//imshow("laneMat", laneMat);

							 if (LANE_DETECTOR){

								 switch (IPM_HK) {

									 case 0: //IPM+HK

									 ProcessLaneImage_IPM(laneMat, laneDetectorConf, startTime, laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, hfLanes, lastHfLanes,
	 			             	lastLateralOffset, lateralOffset, isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone, YAW_ANGLE, PITCH_ANGLE,
	 							      cameraInfo,  lanesConf, IPM_OUT, IPM_cont, particle_detect, particle_track, IPM_HK);
	 										 cv::imshow("IPM_OUT", IPM_OUT);
	 										 cv::imshow("IPM_CONTOUR", IPM_cont);
											 //  cv::imshow("PARTICLE_DETECT", particle_detect);
											 //  cv::imshow("PARTICLE_TRACK", particle_track);
 						 			break;


 						 			case 1:		//IPM+P

									ProcessLaneImage_IPM(laneMat, laneDetectorConf, startTime, laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, hfLanes, lastHfLanes,
				             	lastLateralOffset, lateralOffset, isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone, YAW_ANGLE, PITCH_ANGLE,
								      cameraInfo,  lanesConf, IPM_OUT, IPM_cont, particle_detect, particle_track, IPM_HK);
											//  cv::imshow("IPM_OUT", IPM_OUT);
											//  cv::imshow("IPM_CONTOUR", IPM_cont);
											 cv::imshow("PARTICLE_DETECT", particle_detect);
											 cv::imshow("PARTICLE_TRACK", particle_track);
 						 			break;



 						 			case 2:


										ProcessLaneImage(laneMat, laneDetectorConf, startTime, laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, hfLanes, lastHfLanes,
										lastLateralOffset, lateralOffset, isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone, YAW_ANGLE, PITCH_ANGLE);
										cv::imshow("Lane System", laneMat);

 						 			break;
								 }

            		// if (LANE_DETECTOR && IPM_HK)
            		// {
                // 		ProcessLaneImage_IPM(laneMat, laneDetectorConf, startTime, laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, hfLanes, lastHfLanes,
			          //   	lastLateralOffset, lateralOffset, isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone, YAW_ANGLE, PITCH_ANGLE,
								//      cameraInfo,  lanesConf, IPM_OUT, IPM_cont, particle_detect, particle_track);
								// 		 cv::imshow("IPM_OUT", IPM_OUT);
								// 		 cv::imshow("IPM_CONTOUR", IPM_cont);
								// 		 cv::imshow("PARTICLE_DETECT", particle_detect);
								// 		 cv::imshow("PARTICLE_TRACK", particle_track);
            		// }
								//
								// if (LANE_DETECTOR && HK)
								// {
								// 	ProcessLaneImage(laneMat, laneDetectorConf, startTime, laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, hfLanes, lastHfLanes,
								// 	lastLateralOffset, lateralOffset, isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone, YAW_ANGLE, PITCH_ANGLE);
								// 	cv::imshow("Lane System", laneMat);
								// }
               }

            		if(IMAGE_RECORD)
	         	  	{
                		char *text = new char[100];
                		sprintf(text, LANE_RECORD_IMAGE, idx);
                		cv::imwrite(text, laneMat);
                		delete text;
            		}

            		char *text = new char[30];
            		sprintf(text, "past time: %.2f sec", pastTime);
            		cv::putText(laneMat, text, cv::Point(0, laneMat.rows-5), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0,255,0));
            		cv::putText(laneMat, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 255, 0));
            		delete text;



            		//cv::moveWindow("Lane System", 790, 30);
            		key = cv::waitKey(delay);
            		if (key == 'q' || key == 'Q' || 27 == (int)key) //Esc q\Q\key to stop
                		break;
            		else if(key == 's' || key == 'S')
                    		delay = 0;
            		else
                		delay = 1;

            		/* Update the sampling index */
            //		sampleIdx++;//update the sampling index
            		idx++;

        	}//end while loop

        	laneFeatureFile.close();
        	cv::destroyAllWindows();

        	return 0;
    	}
}//FusedCarSurveillanceSim

#endif //__cplusplus

using LaneDetectorSim::Process;
int main(int argc, const char * argv[])
{

    	return Process(argc, argv);
}
