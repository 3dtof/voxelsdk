/*! 
 * ============================================================================
 *
 * @addtogroup		Horus	
 * @{
 *
 * @file		Horus.cpp
 * @version		1.0
 * @date		1/7/2016
 *
 * @note		People tracking class
 * 
 * Copyright(c) 20015-2016 Texas Instruments Corporation, All Rights Reserved.q
 * TI makes NO WARRANTY as to software products, which are supplied "AS-IS"
 *
 * ============================================================================
 */
#define __HORUS_CPP__
#include "Horus.h"
#include <climits>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>

extern int skipped_frames;

Horus::Horus(int w, int h) : TOFApp(w, h)
{
   _setBackground = false;
   initDisplay();
}

void Horus::initDisplay()
{
   namedWindow( "Draw", WINDOW_NORMAL );

   _ampGain = 100;
   _ampThresh = 3;
   _depthThresh = 2;
   _minContourArea = 100;
   _aspectRatio = 100;
}


void Horus::clipBackground(Mat &dMat, Mat &iMat, float dThr, float iThr)
{
   for (int i = 0; i < dMat.rows; i++) {
      for (int j = 0; j < dMat.cols; j++) {
         float val = (iMat.at<float>(i,j) > iThr && dMat.at<float>(i,j) > dThr) ? 255.0 : 0.0;
         dMat.at<float>(i,j) = val;
      }
   }
}

void Horus::resetBackground()
{
   _setBackground = false;
}


void Horus::getPCA(const vector<cv::Point> &contour, float &center, float &angle)
{
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(contour.size());
    Mat data_pts = Mat(sz, 2, CV_32FC1);
    for (int i = 0; i < data_pts.rows; ++i) {
        data_pts.at<float>(i, 0) = contour[i].x;
        data_pts.at<float>(i, 1) = contour[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the center of the object
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<float>(0, 0)),
                               static_cast<int>(pca_analysis.mean.at<float>(0, 1)));

    //Store the eigenvalues and eigenvectors
    vector<cv::Point2d> eigen_vecs(2);
    vector<float> eigen_val(2);
    for (int i = 0; i < 2; ++i) {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<float>(i, 0), 
                                pca_analysis.eigenvectors.at<float>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<float>(0, i);
    }

    angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
}


// A contour is a person if:
// Aspect ration is largely vertical
// All points are within the same depth interval
// 
bool Horus::isPerson(vector<cv::Point> &contour, Mat dMat)
{
   bool rc = false;
   int area = 0;
   long sumX=0, sumY=0;
   int minX=INT_MAX, minY=INT_MAX;
   int maxX=0, maxY=0;
   int dx, dy;

   // Find biometric statistics
   for (int i=0; i< contour.size(); i++) {
      minX = std::min(minX, contour[i].x);
      minY = std::min(minY, contour[i].y);
      maxX = std::max(maxX, contour[i].x);
      maxY = std::max(maxY, contour[i].y);
      sumX += contour[i].x; 
      sumY += contour[i].y;
   }
   dx = maxX - minX;
   dy = maxY - minY;

   if (contourArea(contour) > _minContourArea) {
      if (dx > 0) {
         float ratio = (float)dy/(float)dx;
         if (ratio > (float)_aspectRatio/100.0) {
            rc = true;
         }
      }
   } 
   
   return rc;
}

static int draw_throttle = 0;

void Horus::update(Frame *frame)
{
   vector< vector<cv::Point> > contours;
   vector<Vec4i> hierarchy;
   RNG rng(12345);
   if (getFrameType() == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME) 
   {
      // Create amplitude and depth Mat
      vector<float> zMap, iMap;
      XYZIPointCloudFrame *frm = dynamic_cast<XYZIPointCloudFrame *>(frame);
      for (int i=0; i< frm->points.size(); i++) {
         zMap.push_back(frm->points[i].z);
         iMap.push_back(frm->points[i].i);
      }
      _iMat = Mat(getDim().height, getDim().width, CV_32FC1, iMap.data());
      _dMat = Mat(getDim().height, getDim().width, CV_32FC1, zMap.data()); 

      // Apply amplitude gain
      _iMat = (float)_ampGain*_iMat;

      // Update background as required
      if (!_setBackground) {
         _dMat.copyTo(_bkgndMat);
         _setBackground = true;
         cout << endl << "Updated background" << endl;
      }

      // Find foreground by subtraction 
      Mat fMat = _bkgndMat-_dMat;

      // Convert to binary image based on amplitude and depth thresholds
      clipBackground(fMat, _iMat, (float)_depthThresh/100.0, (float)_ampThresh/100.0);
      fMat.convertTo(_bMat, CV_8U, 255.0);

      // Apply morphological open to clean up image
      Mat morphMat = _bMat.clone();
      Mat element = getStructuringElement( 0, Size(3,3), cv::Point(1,1) );
      morphologyEx(_bMat, morphMat, 2, element);

      // Find all contours
      findContours(morphMat, contours, hierarchy, CV_RETR_TREE, 
                             CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

      // Draw contours that meet a "person" requirement
      Mat drawing = Mat::zeros(_iMat.size(), CV_8UC3);
      cvtColor(_iMat, drawing, CV_GRAY2RGB);
      
      int peopleCount = 0;
      for ( int i = 0; i < contours.size(); i++ ) { 
         if (isPerson(contours[i], _dMat)) {  
            peopleCount++;
            drawContours( drawing, contours, i, Scalar(0, 0, 255), 2, 8, vector<Vec4i>(), 0, cv::Point() ); 
         }
      }
      putText(drawing, "Cnt="+to_string(peopleCount), cv::Point(40, 30), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0));
      if(skipped_frames == 0) {
        imshow("Draw", drawing);
      } else {
        if((draw_throttle % skipped_frames) == 0) {
          char file_name[80];
          sprintf (file_name, "draw%03d.png", draw_throttle / skipped_frames);
          imwrite (file_name, drawing);
        }
        draw_throttle ++;
      }
   }
}

#undef __HORUS_CPP__
/*! @} */
