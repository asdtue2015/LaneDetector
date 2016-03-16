void mcvInitLaneDetectorConf(char * const fileName,
   LaneDetectorConf *stopLineConf)
{
 //parsed camera data
 LaneDetectorParserInfo stopLineParserInfo;
 //read the data
 assert(LaneDetectorParser_configfile(fileName, &stopLineParserInfo, 0, 1, 1)==0);
 //init the strucure
 stopLineConf->ipmWidth = stopLineParserInfo.ipmWidth_arg;
 stopLineConf->ipmHeight = stopLineParserInfo.ipmHeight_arg;
 stopLineConf->ipmLeft = stopLineParserInfo.ipmLeft_arg;
 stopLineConf->ipmRight = stopLineParserInfo.ipmRight_arg;
 stopLineConf->ipmBottom = stopLineParserInfo.ipmBottom_arg;
 stopLineConf->ipmTop = stopLineParserInfo.ipmTop_arg;
 stopLineConf->ipmInterpolation = stopLineParserInfo.ipmInterpolation_arg;

 stopLineConf->lineWidth = stopLineParserInfo.lineWidth_arg;
 stopLineConf->lineHeight = stopLineParserInfo.lineHeight_arg;
 stopLineConf->kernelWidth = stopLineParserInfo.kernelWidth_arg;
 stopLineConf->kernelHeight = stopLineParserInfo.kernelHeight_arg;
 stopLineConf->lowerQuantile =
     stopLineParserInfo.lowerQuantile_arg;
 stopLineConf->localMaxima =
     stopLineParserInfo.localMaxima_arg;
 stopLineConf->groupingType = stopLineParserInfo.groupingType_arg;
 stopLineConf->binarize = stopLineParserInfo.binarize_arg;
 stopLineConf->detectionThreshold =
     stopLineParserInfo.detectionThreshold_arg;
 stopLineConf->smoothScores =
     stopLineParserInfo.smoothScores_arg;
 stopLineConf->rMin = stopLineParserInfo.rMin_arg;
 stopLineConf->rMax = stopLineParserInfo.rMax_arg;
 stopLineConf->rStep = stopLineParserInfo.rStep_arg;
 stopLineConf->thetaMin = stopLineParserInfo.thetaMin_arg * CV_PI/180;
 stopLineConf->thetaMax = stopLineParserInfo.thetaMax_arg * CV_PI/180;
 stopLineConf->thetaStep = stopLineParserInfo.thetaStep_arg * CV_PI/180;
 stopLineConf->ipmVpPortion = stopLineParserInfo.ipmVpPortion_arg;
 stopLineConf->getEndPoints = stopLineParserInfo.getEndPoints_arg;
 stopLineConf->group = stopLineParserInfo.group_arg;
 stopLineConf->groupThreshold = stopLineParserInfo.groupThreshold_arg;
 stopLineConf->ransac = stopLineParserInfo.ransac_arg;

 stopLineConf->ransacLineNumSamples = stopLineParserInfo.ransacLineNumSamples_arg;
 stopLineConf->ransacLineNumIterations = stopLineParserInfo.ransacLineNumIterations_arg;
 stopLineConf->ransacLineNumGoodFit = stopLineParserInfo.ransacLineNumGoodFit_arg;
 stopLineConf->ransacLineThreshold = stopLineParserInfo.ransacLineThreshold_arg;
 stopLineConf->ransacLineScoreThreshold = stopLineParserInfo.ransacLineScoreThreshold_arg;
 stopLineConf->ransacLineBinarize = stopLineParserInfo.ransacLineBinarize_arg;
 stopLineConf->ransacLineWindow = stopLineParserInfo.ransacLineWindow_arg;

 stopLineConf->ransacSplineNumSamples = stopLineParserInfo.ransacSplineNumSamples_arg;
 stopLineConf->ransacSplineNumIterations = stopLineParserInfo.ransacSplineNumIterations_arg;
 stopLineConf->ransacSplineNumGoodFit = stopLineParserInfo.ransacSplineNumGoodFit_arg;
 stopLineConf->ransacSplineThreshold = stopLineParserInfo.ransacSplineThreshold_arg;
 stopLineConf->ransacSplineScoreThreshold = stopLineParserInfo.ransacSplineScoreThreshold_arg;
 stopLineConf->ransacSplineBinarize = stopLineParserInfo.ransacSplineBinarize_arg;
 stopLineConf->ransacSplineWindow = stopLineParserInfo.ransacSplineWindow_arg;

 stopLineConf->ransacSplineDegree = stopLineParserInfo.ransacSplineDegree_arg;

 stopLineConf->ransacSpline = stopLineParserInfo.ransacSpline_arg;
 stopLineConf->ransacLine = stopLineParserInfo.ransacLine_arg;
 stopLineConf->ransacSplineStep = stopLineParserInfo.ransacSplineStep_arg;

 stopLineConf->overlapThreshold = stopLineParserInfo.overlapThreshold_arg;

 stopLineConf->localizeAngleThreshold = stopLineParserInfo.localizeAngleThreshold_arg;
 stopLineConf->localizeNumLinePixels = stopLineParserInfo.localizeNumLinePixels_arg;

 stopLineConf->extendAngleThreshold = stopLineParserInfo.extendAngleThreshold_arg;
 stopLineConf->extendMeanDirAngleThreshold = stopLineParserInfo.extendMeanDirAngleThreshold_arg;
 stopLineConf->extendLinePixelsTangent = stopLineParserInfo.extendLinePixelsTangent_arg;
 stopLineConf->extendLinePixelsNormal = stopLineParserInfo.extendLinePixelsNormal_arg;
 stopLineConf->extendContThreshold = stopLineParserInfo.extendContThreshold_arg;
 stopLineConf->extendDeviationThreshold = stopLineParserInfo.extendDeviationThreshold_arg;
 stopLineConf->extendRectTop = stopLineParserInfo.extendRectTop_arg;
 stopLineConf->extendRectBottom = stopLineParserInfo.extendRectBottom_arg;

 stopLineConf->extendIPMAngleThreshold = stopLineParserInfo.extendIPMAngleThreshold_arg;
 stopLineConf->extendIPMMeanDirAngleThreshold = stopLineParserInfo.extendIPMMeanDirAngleThreshold_arg;
 stopLineConf->extendIPMLinePixelsTangent = stopLineParserInfo.extendIPMLinePixelsTangent_arg;
 stopLineConf->extendIPMLinePixelsNormal = stopLineParserInfo.extendIPMLinePixelsNormal_arg;
 stopLineConf->extendIPMContThreshold = stopLineParserInfo.extendIPMContThreshold_arg;
 stopLineConf->extendIPMDeviationThreshold = stopLineParserInfo.extendIPMDeviationThreshold_arg;
 stopLineConf->extendIPMRectTop = stopLineParserInfo.extendIPMRectTop_arg;
 stopLineConf->extendIPMRectBottom = stopLineParserInfo.extendIPMRectBottom_arg;

 stopLineConf->splineScoreJitter = stopLineParserInfo.splineScoreJitter_arg;
 stopLineConf->splineScoreLengthRatio = stopLineParserInfo.splineScoreLengthRatio_arg;
 stopLineConf->splineScoreAngleRatio = stopLineParserInfo.splineScoreAngleRatio_arg;
 stopLineConf->splineScoreStep = stopLineParserInfo.splineScoreStep_arg;

 stopLineConf->splineTrackingNumAbsentFrames = stopLineParserInfo.splineTrackingNumAbsentFrames_arg;
 stopLineConf->splineTrackingNumSeenFrames = stopLineParserInfo.splineTrackingNumSeenFrames_arg;

 stopLineConf->mergeSplineThetaThreshold = stopLineParserInfo.mergeSplineThetaThreshold_arg;
 stopLineConf->mergeSplineRThreshold = stopLineParserInfo.mergeSplineRThreshold_arg;
 stopLineConf->mergeSplineMeanThetaThreshold = stopLineParserInfo.mergeSplineMeanThetaThreshold_arg;
 stopLineConf->mergeSplineMeanRThreshold = stopLineParserInfo.mergeSplineMeanRThreshold_arg;
 stopLineConf->mergeSplineCentroidThreshold = stopLineParserInfo.mergeSplineCentroidThreshold_arg;

 stopLineConf->lineTrackingNumAbsentFrames = stopLineParserInfo.lineTrackingNumAbsentFrames_arg;
 stopLineConf->lineTrackingNumSeenFrames = stopLineParserInfo.lineTrackingNumSeenFrames_arg;

 stopLineConf->mergeLineThetaThreshold = stopLineParserInfo.mergeLineThetaThreshold_arg;
 stopLineConf->mergeLineRThreshold = stopLineParserInfo.mergeLineRThreshold_arg;

 stopLineConf->numStrips = stopLineParserInfo.numStrips_arg;


 stopLineConf->checkSplines = stopLineParserInfo.checkSplines_arg;
 stopLineConf->checkSplinesCurvenessThreshold = stopLineParserInfo.checkSplinesCurvenessThreshold_arg;
 stopLineConf->checkSplinesLengthThreshold = stopLineParserInfo.checkSplinesLengthThreshold_arg;
 stopLineConf->checkSplinesThetaDiffThreshold = stopLineParserInfo.checkSplinesThetaDiffThreshold_arg;
 stopLineConf->checkSplinesThetaThreshold = stopLineParserInfo.checkSplinesThetaThreshold_arg;

 stopLineConf->checkIPMSplines = stopLineParserInfo.checkIPMSplines_arg;
 stopLineConf->checkIPMSplinesCurvenessThreshold = stopLineParserInfo.checkIPMSplinesCurvenessThreshold_arg;
 stopLineConf->checkIPMSplinesLengthThreshold = stopLineParserInfo.checkIPMSplinesLengthThreshold_arg;
 stopLineConf->checkIPMSplinesThetaDiffThreshold = stopLineParserInfo.checkIPMSplinesThetaDiffThreshold_arg;
 stopLineConf->checkIPMSplinesThetaThreshold = stopLineParserInfo.checkIPMSplinesThetaThreshold_arg;

 stopLineConf->finalSplineScoreThreshold = stopLineParserInfo.finalSplineScoreThreshold_arg;

 stopLineConf->useGroundPlane = stopLineParserInfo.useGroundPlane_arg;

 stopLineConf->checkColor = stopLineParserInfo.checkColor_arg;
 stopLineConf->checkColorNumBins = stopLineParserInfo.checkColorNumBins_arg;
 stopLineConf->checkColorWindow = stopLineParserInfo.checkColorWindow_arg;
 stopLineConf->checkColorNumYellowMin = stopLineParserInfo.checkColorNumYellowMin_arg;
 stopLineConf->checkColorRGMin = stopLineParserInfo.checkColorRGMin_arg;
 stopLineConf->checkColorRGMax = stopLineParserInfo.checkColorRGMax_arg;
 stopLineConf->checkColorGBMin = stopLineParserInfo.checkColorGBMin_arg;
 stopLineConf->checkColorRBMin = stopLineParserInfo.checkColorRBMin_arg;
 stopLineConf->checkColorRBFThreshold = stopLineParserInfo.checkColorRBFThreshold_arg;
 stopLineConf->checkColorRBF = stopLineParserInfo.checkColorRBF_arg;

 stopLineConf->ipmWindowClear = stopLineParserInfo.ipmWindowClear_arg;;
 stopLineConf->ipmWindowLeft = stopLineParserInfo.ipmWindowLeft_arg;;
 stopLineConf->ipmWindowRight = stopLineParserInfo.ipmWindowRight_arg;;

 stopLineConf->checkLaneWidth = stopLineParserInfo.checkLaneWidth_arg;;
 stopLineConf->checkLaneWidthMean = stopLineParserInfo.checkLaneWidthMean_arg;;
 stopLineConf->checkLaneWidthStd = stopLineParserInfo.checkLaneWidthStd_arg;;
}


void mcvInitCameraInfo (char * const fileName, CameraInfo *cameraInfo)
{
  //parsed camera data
  CameraInfoParserInfo camInfo;
  //read the data
  assert(cameraInfoParser_configfile(fileName, &camInfo, 0, 1, 1)==0);
  //init the strucure
  cameraInfo->focalLength.x = camInfo.focalLengthX_arg;
  cameraInfo->focalLength.y = camInfo.focalLengthY_arg;
  cameraInfo->opticalCenter.x = camInfo.opticalCenterX_arg;
  cameraInfo->opticalCenter.y = camInfo.opticalCenterY_arg;
  cameraInfo->cameraHeight = camInfo.cameraHeight_arg;
  cameraInfo->pitch = camInfo.pitch_arg * CV_PI/180;
  cameraInfo->yaw = camInfo.yaw_arg * CV_PI/180;
  cameraInfo->imageWidth = camInfo.imageWidth_arg;
  cameraInfo->imageHeight = camInfo.imageHeight_arg;
}
