
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
extern const int    IMAGE_RECORD = 1;

/* Run applicaiton */

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



//**********************************
class LDW{


		int  idx;  //index for image sequence
		int  sampleIdx;//    = 1;    //init sampling index
		char laneImg[100];

		double initTime;
		double intervalTime;
		double execTime;  // Execute Time for Each Frame
		double pastTime;
		double lastStartTime;
		char key;
		double delay;
		int IM_RECORD;

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
		double lastLateralOffset;
		double lateralOffset;    // Lateral Offset
		int    detectLaneFlag;   // init state -> normal state 0
		int    isChangeLane;    // Whether lane change happens
		int    changeDone;    // Finish lane change
		int    muWindowSize;    // Initial window size: 5 (sample)
		int    sigmaWindowSize;    // Initial window size: 5 (sample)
		std::vector<float> samplingTime;


		int    laneKalmanIdx;    //Marker of start kalmam

public:
		int Process(int, int, int, double, double);

};



//*********************************

	int LDW::Process(int LANE_DETECTOR, int StartFrame, int EndFrame, double YAW_ANGLE, double PITCH_ANGLE)
	{
		 IM_RECORD        =  IMAGE_RECORD ;
	   idx            = StartFrame;  //index for image sequence
	   sampleIdx  ;//    = 1;    //init sampling index
	   laneImg[100];
		 initTime         = (double)cv::getTickCount();
	   intervalTime     = 0;
		 execTime         = 0;  // Execute Time for Each Frame
		 pastTime         = 0;
		 lastStartTime    = (double)cv::getTickCount();
		 key;
		 delay = 1;
		 /*/* Initialize Lane Kalman Filter */
			cv::KalmanFilter laneKalmanFilter(8, 8, 0);//(rho, theta, delta_rho, delta_theta)x2
			cv::Mat laneKalmanMeasureMat(8, 1, CV_32F, cv::Scalar::all(0));//(rho, theta, delta_rho, delta_theta)

		 		 lastLateralOffset = 0;
		 		 lateralOffset     = 0;    // Lateral Offset
		     detectLaneFlag    = -1;   // init state -> normal state 0
		     isChangeLane      = 0;    // Whether lane change happens
		     changeDone        = 0;    // Finish lane change
		     muWindowSize      = 5;    // Initial window size: 5 (sample)
		     sigmaWindowSize   = 5;    // Initial window size: 5 (sample)



		std::cout << "/*************************************/" << std::endl;
		std::cout << "Input LANE_DETECTOR" << LANE_DETECTOR << std::endl;
		std::cout << "Input StartFrame" << StartFrame << std::endl;
		std::cout << "Input EndFrame" << EndFrame << std::endl;
		std::cout << "Input YAW_ANGLE" << YAW_ANGLE << std::endl;
		std::cout << "Input PITCH_ANGLE" << PITCH_ANGLE << std::endl;
		std::cout << "/*************************************/" << std::endl;

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
                		LaneDetectorSim::ProcessLaneImage(laneMat, laneDetectorConf, startTime, laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, hfLanes, lastHfLanes,
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
            	  cv::waitKey(1);
            		/* Update the sampling index */
            //		sampleIdx++;//update the sampling index
            		idx++;

        	}//end while loop

        	laneFeatureFile.close();
        	cv::destroyAllWindows();

        	return 0;
    	}
//FusedCarSurveillanceSim

#endif //__cplusplus


int main()
{

	int LANE_DETECTOR = 1;
	int StartFrame = 1;
	int EndFrame = 61;
	double YAW_ANGLE = 0.0;
  double PITCH_ANGLE = 0.1;

	LDW object_ldw;
	object_ldw.Process(LANE_DETECTOR, StartFrame, EndFrame, YAW_ANGLE, PITCH_ANGLE);

  return 0;
}
