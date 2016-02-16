
#ifndef LaneDetector_LaneDetectorTools_h
#define LaneDetector_LaneDetectorTools_h

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

namespace LaneDetector {    
    
    void PrintMat(const cv::Mat &mat);
    
    void multiImShow(const std::vector<cv::Mat> &mat, const std::vector<std::string> &winname, cv::Mat &multiMat);
    
    void imShowSub(const std::string &winname, const cv::Mat &mat, 
                   const int &Cols, const int &Rows, const int &winPos);
    
}

#endif
