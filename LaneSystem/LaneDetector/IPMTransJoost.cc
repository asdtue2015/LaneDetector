#define VP_PORTION 0.05


#include "IPMTransJoost.hh"


/*
 We are assuming the world coordinate frame center is at the camera,
 the ground plane is at height -h, the X-axis is going right,
 the Y-axis is going forward, the Z-axis is going up. The
 camera is looking forward with optical axis in direction of
 Y-axis, with possible pitch angle (above or below the Y-axis)
 and yaw angle (left or right).
 The camera coordinates have the same center as the world, but the Xc-axis goes right,
 the  Yc-axis goes down, and the Zc-axis (optical cxis) goes forward. The
 uv-plane of the image is such that u is horizontal going right, v is
 vertical going down.
 The image coordinates uv are such that the pixels are at half coordinates
 i.e. first pixel is (.5,.5) ...etc where the top-left point is (0,0) i.e.
 the tip of the first pixel is (0,0)
*/

/**
 * This function returns the Inverse Perspective Mapping
 * of the input image, assuming a flat ground plane, and
 * given the camera parameters.
 *
 * \param inImage the input image
 * \param outImage the output image in IPM
 * \param ipmInfo the returned IPM info for the transformation
 * \param focalLength focal length (in x and y direction)
 * \param cameraInfo the camera parameters
 * \param outPoints indices of points outside the image
 */
void mcvGetIPM(const CvMat* inImage, CvMat* outImage,
               IPMInfo *ipmInfo, const CameraInfo *cameraInfo,
               list<CvPoint> *outPoints)
{
  //check input images types
  //CvMat inMat, outMat;
  //cvGetMat(inImage, &inMat);
  //cvGetMat(outImage, &outMat);
  //cout << CV_MAT_TYPE(inImage->type) << " " << CV_MAT_TYPE(FLOAT_MAT_TYPE) <<  " " << CV_MAT_TYPE(INT_MAT_TYPE)<<"\n";
  if (!(CV_ARE_TYPES_EQ(inImage, outImage) &&
      (CV_MAT_TYPE(inImage->type)==CV_MAT_TYPE(FLOAT_MAT_TYPE) ||
      (CV_MAT_TYPE(inImage->type)==CV_MAT_TYPE(INT_MAT_TYPE)))))
  {
    cerr << "Unsupported image types in mcvGetIPM";
    exit(1);
  }

  //get size of input image
  FLOAT u, v;
  v = inImage->height;
  u = inImage->width;

  //get the vanishing point
  FLOAT_POINT2D vp;
  vp = mcvGetVanishingPoint(cameraInfo);
  vp.y = MAX(0, vp.y);
  //vp.y = 30;

  //get extent of the image in the xfyf plane
  FLOAT_MAT_ELEM_TYPE eps = ipmInfo->vpPortion * v;//VP_PORTION*v;
  ipmInfo->ipmLeft = MAX(0, ipmInfo->ipmLeft);
  ipmInfo->ipmRight = MIN(u-1, ipmInfo->ipmRight);
  ipmInfo->ipmTop = MAX(vp.y+eps, ipmInfo->ipmTop);
  ipmInfo->ipmBottom = MIN(v-1, ipmInfo->ipmBottom);
  FLOAT_MAT_ELEM_TYPE uvLimitsp[] = {vp.x,
    ipmInfo->ipmRight, ipmInfo->ipmLeft, vp.x,
    ipmInfo->ipmTop, ipmInfo->ipmTop,   ipmInfo->ipmTop,  ipmInfo->ipmBottom};
	//{vp.x, u, 0, vp.x,
	//vp.y+eps, vp.y+eps, vp.y+eps, v};
  CvMat uvLimits = cvMat(2, 4, FLOAT_MAT_TYPE, uvLimitsp);

  //get these points on the ground plane
  CvMat * xyLimitsp = cvCreateMat(2, 4, FLOAT_MAT_TYPE);
  CvMat xyLimits = *xyLimitsp;
  mcvTransformImage2Ground(&uvLimits, &xyLimits,cameraInfo);
  //SHOW_MAT(xyLimitsp, "xyLImits");

  //get extent on the ground plane
  CvMat row1, row2;
  cvGetRow(&xyLimits, &row1, 0);
  cvGetRow(&xyLimits, &row2, 1);
  double xfMax, xfMin, yfMax, yfMin;
  cvMinMaxLoc(&row1, (double*)&xfMin, (double*)&xfMax, 0, 0, 0);
  cvMinMaxLoc(&row2, (double*)&yfMin, (double*)&yfMax, 0, 0, 0);

  INT outRow = outImage->height;
  INT outCol = outImage->width;

  FLOAT_MAT_ELEM_TYPE stepRow = (yfMax-yfMin)/outRow;
  FLOAT_MAT_ELEM_TYPE stepCol = (xfMax-xfMin)/outCol;

  //construct the grid to sample
  CvMat *xyGrid = cvCreateMat(2, outRow*outCol, FLOAT_MAT_TYPE);
  INT i, j;
  FLOAT_MAT_ELEM_TYPE x, y;
  //fill it with x-y values on the ground plane in world frame
  for (i=0, y=yfMax-.5*stepRow; i<outRow; i++, y-=stepRow)
    for (j=0, x=xfMin+.5*stepCol; j<outCol; j++, x+=stepCol)
    {
      CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, i*outCol+j) = x;
      CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, i*outCol+j) = y;
    }
  //get their pixel values in image frame
  CvMat *uvGrid = cvCreateMat(2, outRow*outCol, FLOAT_MAT_TYPE);
  mcvTransformGround2Image(xyGrid, uvGrid, cameraInfo);
  //now loop and find the nearest pixel value for each position
  //that's inside the image, otherwise put it zero
  FLOAT_MAT_ELEM_TYPE ui, vi;
  //get mean of the input image
  CvScalar means = cvAvg(inImage);
  double mean = means.val[0];
  //generic loop to work for both float and int matrix types
  #define MCV_GET_IPM(type) \
  for (i=0; i<outRow; i++) \
      for (j=0; j<outCol; j++) \
      { \
          /*get pixel coordiantes*/ \
          ui = CV_MAT_ELEM(*uvGrid, FLOAT_MAT_ELEM_TYPE, 0, i*outCol+j); \
          vi = CV_MAT_ELEM(*uvGrid, FLOAT_MAT_ELEM_TYPE, 1, i*outCol+j); \
          /*check if out-of-bounds*/ \
          /*if (ui<0 || ui>u-1 || vi<0 || vi>v-1) \*/ \
          if (ui<ipmInfo->ipmLeft || ui>ipmInfo->ipmRight || \
              vi<ipmInfo->ipmTop || vi>ipmInfo->ipmBottom) \
          { \
              CV_MAT_ELEM(*outImage, type, i, j) = (type)mean; \
          } \
          /*not out of bounds, then get nearest neighbor*/ \
          else \
          { \
              /*Bilinear interpolation*/ \
              if (ipmInfo->ipmInterpolation == 0) \
              { \
                  int x1 = int(ui), x2 = int(ui+1); \
                  int y1 = int(vi), y2 = int(vi+1); \
                  float x = ui - x1, y = vi - y1;   \
                  float val = CV_MAT_ELEM(*inImage, type, y1, x1) * (1-x) * (1-y) + \
                      CV_MAT_ELEM(*inImage, type, y1, x2) * x * (1-y) + \
                      CV_MAT_ELEM(*inImage, type, y2, x1) * (1-x) * y + \
                      CV_MAT_ELEM(*inImage, type, y2, x2) * x * y;   \
                  CV_MAT_ELEM(*outImage, type, i, j) =  (type)val; \
  } \
              /*nearest-neighbor interpolation*/ \
              else \
                  CV_MAT_ELEM(*outImage, type, i, j) = \
                      CV_MAT_ELEM(*inImage, type, int(vi+.5), int(ui+.5)); \
          } \
          if (outPoints && \
              (ui<ipmInfo->ipmLeft+10 || ui>ipmInfo->ipmRight-10 || \
              vi<ipmInfo->ipmTop || vi>ipmInfo->ipmBottom-2) )\
              outPoints->push_back(cvPoint(j, i)); \
      }
  if (CV_MAT_TYPE(inImage->type)==FLOAT_MAT_TYPE)
  {
      MCV_GET_IPM(FLOAT_MAT_ELEM_TYPE)
  }
  else
  {
      MCV_GET_IPM(INT_MAT_ELEM_TYPE)
  }

//CvMat our_ipm = *outImage;
//cv::imshow(our_ipm, "IPM image", 10);
//our_ipm = *outImage;
 //our_ipm = outImage;
 //Mat our_ipm=cvarrToMat(outImage);
  //cvSaveImage( "../clips/output_ipm/ipmimage.png", outImage);
  SHOW_IMAGE(outImage, "IPM image", 10);


//  cvSaveImage( "../clips/output_ipm/ipmimage.png", outImage);
   //counter++;
  //cvSaveImage( "../clips/output_ipm/ipmimage_%d.png", our_ipm);

  //return the ipm info
  ipmInfo->xLimits[0] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, 0);
  ipmInfo->xLimits[1] =
    CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 0, (outRow-1)*outCol+outCol-1);
  ipmInfo->yLimits[1] = CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, 0);
  ipmInfo->yLimits[0] =
    CV_MAT_ELEM(*xyGrid, FLOAT_MAT_ELEM_TYPE, 1, (outRow-1)*outCol+outCol-1);
  ipmInfo->xScale = 1/stepCol;
  ipmInfo->yScale = 1/stepRow;
  ipmInfo->width = outCol;
  ipmInfo->height = outRow;

  //clean
  cvReleaseMat(&xyLimitsp);
  cvReleaseMat(&xyGrid);
  cvReleaseMat(&uvGrid);
}


/**
 * Transforms points from the image frame (uv-coordinates)
 * into the real world frame on the ground plane (z=-height)
 *
 * \param inPoints input points in the image frame
 * \param outPoints output points in the world frame on the ground
 *          (z=-height)
 * \param cemaraInfo the input camera parameters
 *
 */
void mcvTransformImage2Ground(const CvMat *inPoints,
                              CvMat *outPoints, const CameraInfo *cameraInfo)
{

  //add two rows to the input points
  CvMat *inPoints4 = cvCreateMat(inPoints->rows+2, inPoints->cols,
      cvGetElemType(inPoints));

  //copy inPoints to first two rows
  CvMat inPoints2, inPoints3, inPointsr4, inPointsr3;
  cvGetRows(inPoints4, &inPoints2, 0, 2);
  cvGetRows(inPoints4, &inPoints3, 0, 3);
  cvGetRow(inPoints4, &inPointsr3, 2);
  cvGetRow(inPoints4, &inPointsr4, 3);
  cvSet(&inPointsr3, cvRealScalar(1));
  cvCopy(inPoints, &inPoints2);
  //create the transformation matrix
  float c1 = cos(cameraInfo->pitch);
  float s1 = sin(cameraInfo->pitch);
  float c2 = cos(cameraInfo->yaw);
  float s2 = sin(cameraInfo->yaw);
  float matp[] = {
    -cameraInfo->cameraHeight*c2/cameraInfo->focalLength.x,
    cameraInfo->cameraHeight*s1*s2/cameraInfo->focalLength.y,
    (cameraInfo->cameraHeight*c2*cameraInfo->opticalCenter.x/
      cameraInfo->focalLength.x)-
      (cameraInfo->cameraHeight *s1*s2* cameraInfo->opticalCenter.y/
      cameraInfo->focalLength.y) - cameraInfo->cameraHeight *c1*s2,

    cameraInfo->cameraHeight *s2 /cameraInfo->focalLength.x,
    cameraInfo->cameraHeight *s1*c2 /cameraInfo->focalLength.y,
    (-cameraInfo->cameraHeight *s2* cameraInfo->opticalCenter.x
      /cameraInfo->focalLength.x)-(cameraInfo->cameraHeight *s1*c2*
      cameraInfo->opticalCenter.y /cameraInfo->focalLength.y) -
      cameraInfo->cameraHeight *c1*c2,

    0,
    cameraInfo->cameraHeight *c1 /cameraInfo->focalLength.y,
    (-cameraInfo->cameraHeight *c1* cameraInfo->opticalCenter.y /
      cameraInfo->focalLength.y) + cameraInfo->cameraHeight *s1,

    0,
    -c1 /cameraInfo->focalLength.y,
    (c1* cameraInfo->opticalCenter.y /cameraInfo->focalLength.y) - s1,
  };
  CvMat mat = cvMat(4, 3, CV_32FC1, matp);
  //multiply
  cvMatMul(&mat, &inPoints3, inPoints4);
  //divide by last row of inPoints4
  for (int i=0; i<inPoints->cols; i++)
  {
    float div = CV_MAT_ELEM(inPointsr4, float, 0, i);
    CV_MAT_ELEM(*inPoints4, float, 0, i) =
        CV_MAT_ELEM(*inPoints4, float, 0, i) / div ;
    CV_MAT_ELEM(*inPoints4, float, 1, i) =
        CV_MAT_ELEM(*inPoints4, float, 1, i) / div;
  }
  //put back the result into outPoints
  cvCopy(&inPoints2, outPoints);
  //clear
  cvReleaseMat(&inPoints4);
}


/**
 * Transforms points from the ground plane (z=-h) in the world frame
 * into points on the image in image frame (uv-coordinates)
 *
 * \param inPoints 2xN array of input points on the ground in world coordinates
 * \param outPoints 2xN output points in on the image in image coordinates
 * \param cameraInfo the camera parameters
 *
 */
void mcvTransformGround2Image(const CvMat *inPoints,
                              CvMat *outPoints, const CameraInfo *cameraInfo)
{
  //add two rows to the input points
  CvMat *inPoints3 = cvCreateMat(inPoints->rows+1, inPoints->cols,
      cvGetElemType(inPoints));

  //copy inPoints to first two rows
  CvMat inPoints2,  inPointsr3;
  cvGetRows(inPoints3, &inPoints2, 0, 2);
  cvGetRow(inPoints3, &inPointsr3, 2);
  cvSet(&inPointsr3, cvRealScalar(-cameraInfo->cameraHeight));
  cvCopy(inPoints, &inPoints2);
  //create the transformation matrix
  float c1 = cos(cameraInfo->pitch);
  float s1 = sin(cameraInfo->pitch);
  float c2 = cos(cameraInfo->yaw);
  float s2 = sin(cameraInfo->yaw);
  float matp[] = {
    cameraInfo->focalLength.x * c2 + c1*s2* cameraInfo->opticalCenter.x,
    -cameraInfo->focalLength.x * s2 + c1*c2* cameraInfo->opticalCenter.x,
    - s1 * cameraInfo->opticalCenter.x,

    s2 * (-cameraInfo->focalLength.y * s1 + c1* cameraInfo->opticalCenter.y),
    c2 * (-cameraInfo->focalLength.y * s1 + c1* cameraInfo->opticalCenter.y),
    -cameraInfo->focalLength.y * c1 - s1* cameraInfo->opticalCenter.y,

    c1*s2,
    c1*c2,
    -s1
  };
  CvMat mat = cvMat(3, 3, CV_32FC1, matp);
  //multiply
  cvMatMul(&mat, inPoints3, inPoints3);
  //divide by last row of inPoints4
  for (int i=0; i<inPoints->cols; i++)
  {
    float div = CV_MAT_ELEM(inPointsr3, float, 0, i);
    CV_MAT_ELEM(*inPoints3, float, 0, i) =
        CV_MAT_ELEM(*inPoints3, float, 0, i) / div ;
    CV_MAT_ELEM(*inPoints3, float, 1, i) =
        CV_MAT_ELEM(*inPoints3, float, 1, i) / div;
  }
  //put back the result into outPoints
  cvCopy(&inPoints2, outPoints);
  //clear
  cvReleaseMat(&inPoints3);
}


/**
 * Computes the vanishing point in the image plane uv. It is
 * the point of intersection of the image plane with the line
 * in the XY-plane in the world coordinates that makes an
 * angle yaw clockwise (form Y-axis) with Y-axis
 *
 * \param cameraInfo the input camera parameter
 *
 * \return the computed vanishing point in image frame
 *
 */
FLOAT_POINT2D mcvGetVanishingPoint(const CameraInfo *cameraInfo)
{
  //get the vp in world coordinates
  FLOAT_MAT_ELEM_TYPE vpp[] = {sin(cameraInfo->yaw)/cos(cameraInfo->pitch),
          cos(cameraInfo->yaw)/cos(cameraInfo->pitch), 0};
  CvMat vp = cvMat(3, 1, FLOAT_MAT_TYPE, vpp);

  //transform from world to camera coordinates
  //
  //rotation matrix for yaw
  FLOAT_MAT_ELEM_TYPE tyawp[] = {cos(cameraInfo->yaw), -sin(cameraInfo->yaw), 0,
                sin(cameraInfo->yaw), cos(cameraInfo->yaw), 0,
                0, 0, 1};
  CvMat tyaw = cvMat(3, 3, FLOAT_MAT_TYPE, tyawp);
  //rotation matrix for pitch
  FLOAT_MAT_ELEM_TYPE tpitchp[] = {1, 0, 0,
                  0, -sin(cameraInfo->pitch), -cos(cameraInfo->pitch),
                  0, cos(cameraInfo->pitch), -sin(cameraInfo->pitch)};
  CvMat transform = cvMat(3, 3, FLOAT_MAT_TYPE, tpitchp);
  //combined transform
  cvMatMul(&transform, &tyaw, &transform);

  //
  //transformation from (xc, yc) in camra coordinates
  // to (u,v) in image frame
  //
  //matrix to shift optical center and focal length
  FLOAT_MAT_ELEM_TYPE t1p[] = {
    cameraInfo->focalLength.x, 0,
    cameraInfo->opticalCenter.x,
    0, cameraInfo->focalLength.y,
    cameraInfo->opticalCenter.y,
    0, 0, 1};
  CvMat t1 = cvMat(3, 3, FLOAT_MAT_TYPE, t1p);
  //combine transform
  cvMatMul(&t1, &transform, &transform);
  //transform
  cvMatMul(&transform, &vp, &vp);

  //
  //clean and return
  //
  FLOAT_POINT2D ret;
  ret.x = cvGetReal1D(&vp, 0);
  ret.y = cvGetReal1D(&vp, 1);
  return ret;
}

/******this might be shifted to somewhere else, take a look later */

void mcvGetLanes(const CvMat *inImage, CameraInfo *cameraInfo, LaneDetectorConf_J *stopLineConf)
{
  //input size
  CvSize inSize = cvSize(inImage->width, inImage->height);

  //TODO: smooth image
  CvMat *image = cvCloneMat(inImage);
  //cvSmooth(image, image, CV_GAUSSIAN, 5, 5, 1, 1);

  //SHOW_IMAGE(image, "Input image", 10);

  IPMInfo ipmInfo;

  //state: create a new structure, and put pointer to it if it's null
  // LineState newState;
  // if(!state) state = &newState;

//     //get the IPM size such that we have height of the stop line
//     //is 3 pixels
//     double ipmWidth, ipmHeight;
//     mcvGetIPMExtent(cameraInfo, &ipmInfo);
//     ipmHeight = 3*(ipmInfo.yLimits[1]-ipmInfo.yLimits[0]) / (stopLineConf->lineHeight/3.);
//     ipmWidth = ipmHeight * 4/3;
//     //put into the conf
//     stopLineConf->ipmWidth = int(ipmWidth);
//     stopLineConf->ipmHeight = int(ipmHeight);

//     #ifdef DEBUG_GET_STOP_LINES
//     cout << "IPM width:" << stopLineConf->ipmWidth << " IPM height:"
// 	 << stopLineConf->ipmHeight << "\n";
//     #endif


  //Get IPM
  CvSize ipmSize = cvSize((int)stopLineConf->ipmWidth,
      (int)stopLineConf->ipmHeight);
  CvMat * ipm;
  ipm = cvCreateMat(ipmSize.height, ipmSize.width, inImage->type);
  //mcvGetIPM(inImage, ipm, &ipmInfo, cameraInfo);
  ipmInfo.vpPortion = stopLineConf->ipmVpPortion;
  ipmInfo.ipmLeft = stopLineConf->ipmLeft;
  ipmInfo.ipmRight = stopLineConf->ipmRight;
  ipmInfo.ipmTop = stopLineConf->ipmTop;
  ipmInfo.ipmBottom = stopLineConf->ipmBottom;
  ipmInfo.ipmInterpolation = stopLineConf->ipmInterpolation;
  list<CvPoint> outPixels;
  list<CvPoint>::iterator outPixelsi;
  mcvGetIPM(image, ipm, &ipmInfo, cameraInfo, &outPixels);


}
void processJ(CvMat *laneMat, LaneDetector_J:: CameraInfo &cameraInfo, LaneDetector_J::LaneDetectorConf_J &lanesConf)
{

CvMat *raw_mat, *mat;
mcvLoadImage(&laneMat, &raw_mat, &mat);

// detect lanes
vector<FLOAT> lineScores, splineScores;
vector<Line> lanes;
mcvGetLanes(mat, raw_mat, &lanes, &cameraInfo, &lanesConf, NULL);

}




void mcvLoadImage(const cvMat* ipminputimage , CvMat **clrImage, CvMat** channelImage)
{
  // load the image
  cvMat im;
  im = ipminputimage;
  // convert to mat and get first channel
  CvMat temp;
  cvGetMat(im, &temp);
  *clrImage = cvCloneMat(&temp);
  // convert to single channel
  CvMat *schannel_mat;
  CvMat* tchannelImage = cvCreateMat(im->height, im->width, INT_MAT_TYPE);
  cvSplit(*clrImage, tchannelImage, NULL, NULL, NULL);
  // convert to float
  *channelImage = cvCreateMat(im->height, im->width, FLOAT_MAT_TYPE);
  cvConvertScale(tchannelImage, *channelImage, 1./255);
  // destroy
  cvReleaseMat(&tchannelImage);
  cvReleaseImage(&im);
}


void SHOW_IMAGE(const CvMat *pmat, const char str[], int wait)
{
  //cout << "channels:" << CV_MAT_CN(pmat->type) << "\n";
  //scale it
  //CvMat *mat = cvCreateMat(pmat->height, pmat->width, pmat->type);
  //cvCopy(pmat, mat);
  CvMat *mat = cvCloneMat(pmat);//->rows, pmat->cols, INT_MAT_TYPE);//cvCloneMat(pmat);
  assert(mat);
  //convert to int type
  //cvConvert(pmat, mat);
  if(CV_MAT_CN(mat->type) == 1)//FLOAT_MAT_TYPE)
    mcvScaleMat(mat, mat);
  //show it
  //cout << "in\n";
  cvNamedWindow(str, CV_WINDOW_AUTOSIZE); //0 1
  cvShowImage(str, mat);

  //cvSaveImage( "../clips/output_ipm/ipmimage_.png", pmat);
  cvWaitKey(wait);
  //cvDestroyWindow(str);
  //clear
  cvReleaseMat(&mat);
  //cout << "out\n";
}
