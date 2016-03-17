/**
 * \file LaneDetector.hh
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date Thu 26 Jul, 2007
 *
 */

#ifndef INIT_HH_
#define INIT_HH_

#ifdef __cplusplus
extern "C" {
#endif

#include "IPMTransJoost.hh"

namespace LaneDetector_J
{

#define GROUPING_TYPE_HV_LINES 0
#define GROUPING_TYPE_HOUGH_LINES 1

///Structure to hold lane detector settings
typedef struct LaneDetectorConf_J
{
  ///width of IPM image to use
  FLOAT ipmWidth;
  ///height of IPM image
  FLOAT ipmHeight;
  ///Left point in original image of region to make IPM for
  int ipmLeft;
  ///Right point in original image of region to make IPM for
  int ipmRight;
  ///Top point in original image of region to make IPM for
  int ipmTop;
  ///Bottom point in original image of region to make IPM for
  int ipmBottom;
  ///The method to use for IPM interpolation
  int ipmInterpolation;

  ///width of line we are detecting
  FLOAT lineWidth;
  ///height of line we are detecting
  FLOAT lineHeight;
  ///kernel size to use for filtering
  unsigned char kernelWidth;
  unsigned char kernelHeight;
  ///lower quantile to use for thresholding the filtered image
  FLOAT lowerQuantile;
  ///whether to return local maxima or just the maximum
  bool localMaxima;
  ///the type of grouping to use: 0 for HV lines and 1 for Hough Transform
  unsigned char groupingType;
  ///whether to binarize the thresholded image or use the
  ///raw filtered image
  bool binarize;
  //unsigned char topClip;
  ///threshold for line scores to declare as line
  FLOAT detectionThreshold;
  ///whtehter to smooth the line scores detected or not
  bool smoothScores;
  ///rMin, rMax and rStep for Hough Transform (pixels)
  float rMin, rMax, rStep;
  ///thetaMin, thetaMax, thetaStep for Hough Transform (radians)
  float thetaMin, thetaMax, thetaStep;
  ///portion of image height to add to y-coordinate of vanishing
  ///point when computing the IPM image
  float ipmVpPortion;
  ///get end points or not
  bool getEndPoints;
  ///group nearby lines
  bool group;
  ///threshold for grouping nearby lines
  float groupThreshold;
  ///use RANSAC or not
  bool ransac;
  ///RANSAC Line parameters
  int ransacLineNumSamples;
  int ransacLineNumIterations;
  int ransacLineNumGoodFit;
  float ransacLineThreshold;
  float ransacLineScoreThreshold;
  bool ransacLineBinarize;
  ///half width to use for ransac window
  int ransacLineWindow;
  ///RANSAC Spline parameters
  int ransacSplineNumSamples;
  int ransacSplineNumIterations;
  int ransacSplineNumGoodFit;
  float ransacSplineThreshold;
  float ransacSplineScoreThreshold;
  bool ransacSplineBinarize;
  int ransacSplineWindow;
  ///degree of spline to use
  int ransacSplineDegree;
  ///use a spline or straight line
  bool ransacSpline;
  bool ransacLine;

  ///step used to pixelize spline in ransac
  float ransacSplineStep;

  ///Overlap threshold to use for grouping of bounding boxes
  float overlapThreshold;

  ///Angle threshold used for localization (cosine, 1: most restrictive,
  /// 0: most liberal)
  float localizeAngleThreshold;
  ///Number of pixels to go in normal direction for localization
  int localizeNumLinePixels;

  ///Angle threshold used for extending (cosine, 1: most restrictive,
  /// 0: most liberal)
  float extendAngleThreshold;
  ///Angle threshold from mean direction used for extending (cosine, 1:
  /// most restrictive, 0: most liberal)
  float extendMeanDirAngleThreshold;
  ///Number of pixels to go in tangent direction for extending
  int extendLinePixelsTangent;
  ///Number of pixels to go in normal direction for extending
  int extendLinePixelsNormal;
  ///Trehsold used for stopping the extending process (higher ->
  /// less extending)
  float extendContThreshold;
  ///Stop extending when number of deviating points exceeds this threshold
  int extendDeviationThreshold;
  ///Top point for extension bounding box
  int extendRectTop;
  ///	Bottom point for extension bounding box
  int extendRectBottom;

  ///Angle threshold used for extending (cosine, 1: most restrictive,
  /// 0: most liberal)
  float extendIPMAngleThreshold;
  ///Angle threshold from mean direction used for extending (cosine,
  /// 1: most restrictive, 0: most liberal)
  float extendIPMMeanDirAngleThreshold;
  ///Number of pixels to go in tangent direction for extending
  int extendIPMLinePixelsTangent;
  ///Number of pixels to go in normal direction for extending
  int extendIPMLinePixelsNormal;
  ///Trehsold used for stopping the extending process (higher ->
  /// less extending)
  float emcvInitCameraInfoxtendIPMContThreshold;
  ///Stop extending when number of deviating points exceeds this threshold
  int extendIPMDeviationThreshold;
  ///Top point for extension bounding box
  int extendIPMRectTop;
  ///	Bottom point for extension bounding box
  int extendIPMRectBottom;


  ///Number of pixels to go around the spline to compute score
  int splineScoreJitter;
  ///Ratio of spline length to use
  float splineScoreLengthRatio;
  ///Ratio of spline angle to use
  float splineScoreAngleRatio;
  ///Step to use for spline score computation
  float splineScoreStep;

  ///number of frames the track is allowed to be absent before deleting it
  int splineTrackingNumAbsentFrames;
  ///number of frames before considering the track good
  int splineTrackingNumSeenFrames;

  ///Angle threshold for merging splines (radians)
  float mergeSplineThetaThreshold;
  ///R threshold (distance from origin) for merginn splines
  float mergeSplineRThreshold;
  ///Mean Angle threshold for merging splines (radians)
  float mergeSplineMeanThetaThreshold;
  ///Mean R threshold (distance from origin) for merginn splines
  float mergeSplineMeanRThreshold;
  ///Distance threshold between spline cetroids for merging
  float mergeSplineCentroidThreshold;

  ///number of frames the track is allowed to be absent before deleting it
  int lineTrackingNumAbsentFrames;
  ///number of frames before considering the track good
  int lineTrackingNumSeenFrames;

  ///Angle threshold for merging lines (radians)
  float mergeLineThetaThreshold;
  ///R threshold (distance from origin) for merging lines
  float mergeLineRThreshold;

  ///Number of horizontal strips to divide the image to
  int numStrips;

  ///Whtethet to check splines or not
  bool checkSplines;
  ///Curveness Threshold for checking splines
  float checkSplinesCurvenessThreshold;
  ///Length Threshold for checking splines
  float checkSplinesLengthThreshold;
  ///ThetaDiff Threshold for checking splines
  float checkSplinesThetaDiffThreshold;
  ///ThetaThreshold Threshold for checking splines
  float checkSplinesThetaThreshold;

  ///Whtethet to check IPM splines or not
  bool checkIPMSplines;
  ///Curveness Threshold for checking splines
  float checkIPMSplinesCurvenessThreshold;
  ///Length Threshold for checking splines
  float checkIPMSplinesLengthThreshold;
  ///ThetaDiff Threshold for checking splines
  float checkIPMSplinesThetaDiffThreshold;
  ///ThetaThreshold Threshold for checking splines
  float checkIPMSplinesThetaThreshold;

  ///Final Threshold for declaring a valid spline
  float finalSplineScoreThreshold;

  ///Use ground plane when sending to map or not
  bool useGroundPlane;

  ///Whether to check colors or not
  bool checkColor;
  ///Size of window to use
  int checkColorWindow;
  ///Number of bins to use for histogram
  int checkColorNumBins;
  ///Min ratio of yellow points
  float checkColorNumYellowMin;
  ///Min RG diff
  float checkColorRGMin;
  ///Max RG diff
  float checkColorRGMax;
  ///Min GB diff
  float checkColorGBMin;
  ///Min RB diff
  float checkColorRBMin;
  ///RBF Threshold
  float checkColorRBFThreshold;
  ///Whether to use RBF or not
  bool checkColorRBF;

  ///Whether to clear part of the IPM image
  bool ipmWindowClear;
  ///Left corrdinate of window to keep in IPM
  int ipmWindowLeft;
  ///Left corrdinate of window to keep in IPM
  int ipmWindowRight;

  ///Whether to check lane width or not
  bool checkLaneWidth;
  ///Mean of lane width to look for
  float checkLaneWidthMean;
  ///Std deviation of lane width to look for
  float checkLaneWidthStd;
} LaneDetectorConf_J;


///Camera Calibration info
typedef struct CameraInfo
{
  ///focal length in x and y
  FLOAT_POINT2D focalLength;
  ///optical center coordinates in image frame (origin is (0,0) at top left)
  FLOAT_POINT2D opticalCenter;
  ///height of camera above ground
  FLOAT cameraHeight;
  ///pitch angle in radians (+ve downwards)
  FLOAT pitch;
  ///yaw angle in radians (+ve clockwise)
  FLOAT yaw;
  ///width of images
  FLOAT imageWidth;
  ///height of images
  FLOAT imageHeight;
}CameraInfo;

typedef struct IPMInfo
{
  ///min and max x-value on ground in world coordinates
  FLOAT xLimits[2];
  ///min and max y-value on ground in world coordinates
  FLOAT yLimits[2];
  ///conversion between mm in world coordinate on the ground
  ///in x-direction and pixel in image
  FLOAT xScale;
  ///conversion between mm in world coordinate on the ground
  ///in y-direction and pixel in image
  FLOAT yScale;
  ///width
  int width;
  ///height
  int height;
  ///portion of image height to add to y-coordinate of
  ///vanishing point
  float vpPortion;
  ///Left point in original image of region to make IPM for
  float ipmLeft;
  ///Right point in original image of region to make IPM for
  float ipmRight;
  ///Top point in original image of region to make IPM for
  float ipmTop;
  ///Bottom point in original image of region to make IPM for
  float ipmBottom;
  ///interpolation to use for IPM (0: bilinear, 1:nearest neighbor)
  int ipmInterpolation;
}IPMInfo;


//function definitions

/** This initializes the LaneDetectorinfo structure
 *
 * \param fileName the input file name
 * \param stopLineConf the structure to fill
 *
 *
 */
void mcvInitLaneDetectorConf(char * const fileName, LaneDetectorConf_J *stopLineConf);
void mcvInitCameraInfo (char * const fileName, CameraInfo *cameraInfo);


} // namespace LaneDetector

#ifdef __cplusplus
}
#endif
#endif /*INIT_HH_HH_*/
