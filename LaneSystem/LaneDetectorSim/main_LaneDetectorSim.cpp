
#include "main_LaneDetectorSim.h"
#include "Process_LaneDetectorSim.h"
#include <stdexcept>

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
extern const char   LANE_RAW_NAME[]     = "./inputdata/cropped_images/cropped_%d.png";
//extern const char   LANE_RAW_NAME[]     = "./inputdata/washington/lane_%d.png";
// extern const char   LANE_RAW_NAME[]     = "/home/lixp/Developer/KIT/2011_09_26/2011_09_26_drive_0015_sync/image_00/data/%010d.png";
// extern const char   LANE_RAW_NAME[]     = "/home/lixp/Developer/Data/LaneRaw_10-07-2013_18h30m21s/lane_%d.jpg";


extern const int    TH_KALMANFILTER     = 1; // originally 1

namespace LaneDetectorSim {

	int Process(int argc, const char* argv[])
	{
        	if(argc < 5)
            	std::cout << "Not enough parameters" << std::endl;

		int	LANE_DETECTOR  	= atoi(argv[1]);
		int	StartFrame    	= atoi(argv[2]); // FRAME_START
		int EndFrame	    	= atoi(argv[3]); // FRAME_END
    double YAW_ANGLE    = atof(argv[4]); // yaw - X
    double PITCH_ANGLE  = atof(argv[5]); // pitch - Y

		std::cout << "/*************************************/" << std::endl;
		std::cout << "Input LANE_DETECTOR" << LANE_DETECTOR << std::endl;
		std::cout << "Input StartFrame" << StartFrame << std::endl;
		std::cout << "Input EndFrame" << EndFrame << std::endl;
		std::cout << "Input YAW_ANGLE" << YAW_ANGLE << std::endl;
		std::cout << "Input PITCH_ANGLE" << PITCH_ANGLE << std::endl;
		std::cout << "/*************************************/" << std::endl;


		int  idx            = StartFrame;  //index for image sequence
		int  sampleIdx      = 1;    //init sampling index
		char laneImg[100];

		double initTime         = (double)cv::getTickCount();
		double intervalTime     = 0;
		double execTime         = 0;  // Execute Time for Each Frame
		double pastTime         = 0;
		double lastStartTime    = (double)cv::getTickCount();
		char key;
		double delay = 1;

		std::ofstream laneFeatureFile;

		/* Parameters for Lane Detector */
		cv::Mat laneMat;
		LaneDetector::LaneDetectorConf laneDetectorConf;
		std::vector<cv::Vec2f> hfLanes;
		std::vector<cv::Vec2f> lastHfLanes;
		std::vector<cv::Vec2f> preHfLanes;

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

        	if (LANE_DETECTOR)
	      	{
             		/* Lane detect and tracking */
            		sprintf(laneImg, LANE_RAW_NAME , idx);
            		laneMat = cv::imread(laneImg);
            		LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2); // KIT 1, ESIEE 2
            		LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
        	}

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

            		if (LANE_DETECTOR)
            		{
                		ProcessLaneImage(laneMat, laneDetectorConf, startTime, laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, hfLanes, lastHfLanes,
			            	lastLateralOffset, lateralOffset, isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone, YAW_ANGLE, PITCH_ANGLE);
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

            		cv::imshow("Lane System", laneMat);
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
