/*! 
 * ============================================================================
 *
 * @addtogroup		Horus
 * @{
 *
 * @file		Horus.h
 * @version		1.0
 * @date		12/14/2015
 *
 * @note		People tracking class
 * 
 * Copyright(c) 2007-2012 Texas Instruments Corporation, All Rights Reserved.
 * TI makes NO WARRANTY as to software products, which are supplied "AS-IS"
 *
 * ============================================================================
 */
#include "TOFApp.h"
#include <math.h>

#ifndef __HORUS_H__
#define __HORUS_H__

class Horus : public TOFApp
{
public:
   Horus(int w, int h);
   void update(Frame *frm);
   void resetBackground();
   void initDisplay();

private:
   Mat _dMat, _iMat, _bMat, _bkgndMat;
   bool _setBackground;
   int _depthThresh;
   int _ampGain;
   int _ampThresh;
   int _minContourArea;
   int _aspectRatio;
   HOGDescriptor hog;

private:
   bool isPerson(vector<cv::Point> &contour, Mat dMat);
   void clipBackground(Mat &dMat, Mat &iMat, float dThr, float iThr);
   void getPCA(const vector<cv::Point> &contour, float &center, float &angle);
};

#endif // __HORUS_H__
/*! @} */

