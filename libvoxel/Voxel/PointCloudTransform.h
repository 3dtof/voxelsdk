/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_POINTCLOUD_TRANSFORM_H
#define VOXEL_POINTCLOUD_TRANSFORM_H

#include "Point.h"
#include "Common.h"
#include "Frame.h"

#include "VoxelExports.h"

#include <float.h>

#define POINT_INVALID FLT_MAX

namespace Voxel
{
/**
 * \addtogroup Frm
 * @{
 */

class VOXEL_EXPORT PointCloudTransform
{
public:
  uint32_t left, top;              // Image top-left corner
  uint32_t width, height;            // sensor width and height
  uint32_t rowsToMerge, columnsToMerge; // Binning
  float fx, fy;             // x- and y- focal distance
  float cx, cy;             // image center
  float k1, k2, k3;           // radial distortion parameters
  float p1, p2;             // tangential distortion parameters
  
  Vector<Point> directions; // Directional array

  // Clippings
  Point leftClippingNormal;
  Point rightClippingNormal;
  Point topClippingNormal;
  Point bottomClippingNormal;

  // Estimation errors
  float fxError, fyError;
  float cxError, cyError;
  float k1Error, k2Error, k3Error;
  float p1Error, p2Error;
  float pixelErrorX, pixelErrorY;

  // Checkerboard information
  int nCornersPerImages;
  int nImages;

  Point &getDirection(int row, int col);

public:
  PointCloudTransform(uint32_t left, uint32_t top, uint32_t width, uint32_t height, 
                      uint32_t rowsToMerge, uint32_t columnsToMerge,
                      float fx, float fy, float cx, float cy,
                      float k1, float k2, float k3, float p1, float p2);
  
  void calcAperatureAngleRadians(int &horizontalAperatureAngle, float &verticalAperatureAngle);
  void rescaleParameters(int width, int height);
  Point worldToImage(const Point &p);
  Point imageToWorld(const Point &p, float depth);
  
  bool depthToPointCloud(const Vector<float> &distances, PointCloudFrame &pointCloudFrame);
  
private:
  Point _screenToNormalizedScreen(const Point &screen, bool verify);
  Point _normalizedScreenToScreen(const Point &normalizedScreen);
  Point _lensCorrection(const Point &normalizedScreen);
  bool _computeConcaveMirrorBorders(int width, int height, double &minx, double &maxX,
                                      double &minY, double &maxY);
  bool _computeConcaveMirrorBordersOuter(int width, int height, double &minX, double &maxX,
                                          double &minY, double &maxY);
  void _computeConcaveMirror(int width, int height, 
                             Vector<double> &leftArr, Vector<double> &rightArr,
                             Vector<double> &topArr, Vector<double> &bottomArr);
  
  void _init();
  
  Point _normalizedScreenToUnitWorld(const Point &normalizedScreen);
  void _computeClippingPlanes();
};

typedef Ptr<PointCloudTransform> PointCloudTransformPtr;

/**
 * @}
 */

}

#endif // VOXEL_POINTCLOUD_TRANSFORM_H
