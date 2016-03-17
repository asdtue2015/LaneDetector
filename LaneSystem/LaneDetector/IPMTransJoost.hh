/***
 * \file InversePerspectiveMapping.hh
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date 11/29/2006
 */

#ifndef IPMTransJoost_HH_
#define IPMTransJoost_HH_


#include "cv.h"
#include "mcv.hh"
#include <list>


using namespace std;

namespace LaneDetector
{
//functions definitions
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
 */
void mcvGetIPM(const CvMat* inImage, CvMat* outImage, IPMInfo *ipmInfo, const CameraInfo *cameraInfo, list<CvPoint> *outPoints);

/**
 * Transforms points from the image frame (uv-coordinates)
 * into the real world frame on the ground plane (z=-height)
 *
 * \param inPoints input points in the image frame (2xN matrix)
 * \param outPoints output points in the world frame on the ground
 *          (z=-height) (2xN matrix with xw, yw and implicit z=-height)
 * \param cemaraInfo the input camera parameters
 *
 */
void mcvTransformImage2Ground(const CvMat *inPoints, CvMat *outPoints, const CameraInfo *cameraInfo);

/**
 * Transforms points from the ground plane (z=-h) in the world frame
 * into points on the image in image frame (uv-coordinates)
 *
 * \param inPoints 2xN array of input points on the ground in world coordinates
 * \param outPoints 2xN output points in on the image in image coordinates
 * \param cameraInfo the camera parameters
 *
 */
void mcvTransformGround2Image(const CvMat *inPoints, CvMat *outPoints, const CameraInfo *cameraInfo);

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
FLOAT_POINT2D mcvGetVanishingPoint(const CameraInfo *cameraInfo);

void mcvGetLanes(const CvMat *inImage, CameraInfo *cameraInfo, LaneDetectorConf *stopLineConf);

void processJ(CvMat *laneMat, CameraInfo& cameraInfo, LaneDetectorConf& lanesConf);

void mcvLoadImage(const cvMat* ipminputimage , CvMat **clrImage, CvMat** channelImage);

void SHOW_IMAGE(const CvMat *pmat, const char str[]="Window", int wait=0);

/**
 * Gets the extent of the image on the ground plane given the camera parameters
 *
 * \param cameraInfo the input camera info
 * \param ipmInfo the IPM info containing the extent on ground plane:
 *  xLimits & yLimits only are changed
 *
 */
// void mcvGetIPMExtent(const CameraInfo *cameraInfo, IPMInfo *ipmInfo);

} // namespace LaneDetector

#endif /*IPMTransJoost_HH_*/
