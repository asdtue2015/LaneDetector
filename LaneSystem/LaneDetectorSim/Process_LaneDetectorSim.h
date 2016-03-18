
#ifndef LaneDetectorSim_Process_h
#define LaneDetectorSim_Process_h

#include "../LaneDetector/DetectLanes.h"
#include "../LaneDetector/TrackLanes.h"
#include "../LaneDetector/LaneDetectorTools.h"
#include "../LaneDetector/GenerateLaneIndicators.h"
#include "../LaneDetector/IPMTransformation.h"

//extern "C" {
#include "../LaneDetector/IPMTransJoost.hh"
//}

#include "cmdline.h"
//extern "C" {
//#include "../LaneDetector/init.hh"
//}
//*****************************************************
// #include "../JoostFiles/mcv.hh"
// #include "../JoostFiles/InversePerspectiveMapping.hh"
// #include "../JoostFiles/LaneDetectorOpt.h"
// #include "../JoostFiles/ranker.h"
// #include <cv.h>
// #include "../JoostFiles/LaneDetector.hh"
//****************************************************
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <vector>
#include <iterator>
#include <opencv2/opencv.hpp>

//!IPC
#include <sys/types.h>
#include <sys/msg.h>
#include <sys/ipc.h>
#include "errno.h"

namespace LaneDetectorSim{
    const int NUM_LANE = 17;
    const std::string laneFeatureName[NUM_LANE] =
    {
        "Frame",
        "LO", "LATSD", "LATSD_b",
        "LATMEAN", "LATMEAN_b",
        "LANEDEV", "LANEDEV_b",
        "LANEX",
        "TLC", "TLC_2s","TLCF_2s", "TLC_halfs", "TLCF_halfs", "TLC_min",
        // "TOT",
        "execTime", "pastTime"
    };

    void ProcessLaneImage(cv::Mat &laneMat,
                          LaneDetector::LaneDetectorConf &laneDetectorConf,
                          const double &startTime,
                          cv::KalmanFilter &laneKalmanFilter,
                          cv::Mat &laneKalmanMeasureMat, int &laneKalmanIdx,
                          std::vector<cv::Vec2f> &hfLanes,
                          std::vector<cv::Vec2f> &lastHfLanes,
                          double & lastLateralOffset,
                          double &lateralOffset, int &isChangeLane,
                          int &detectLaneFlag,  const int &idx, double &execTime,
                          std::vector<cv::Vec2f> &preHfLanes, int &changeDone,
                          const double &YAW_ANGLE, const double &PITCH_ANGLE,
                          LaneDetector_J::CameraInfo &cameraInfo, LaneDetector_J::LaneDetectorConf_J &lanesConf);

    void GetSamplingTime(const char *fileName, std::vector<float> &samplingTime);

    void InitRecordData(std::ofstream &file, const char* fileName, const std::string *strName, const int &elemNum);

    void RecordLaneFeatures(std::ofstream &file, const LaneDetector::LaneFeature &laneFeatures,
                        const double &execTime, const double &pastTime);

    void CodeMsg( const LaneDetector::LaneFeature &laneFeatures, char *str);
}
#endif
