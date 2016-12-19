/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "PointCloudTransform.h"

#include <limits>
#include <algorithm>

#include "Logger.h"

#define _USE_MATH_DEFINES
#include <math.h>
#ifdef ARM_OPT
#include <arm_neon.h>
#endif

namespace Voxel
{

Point &PointCloudTransform::getDirection(int row, int col)
{
  return directions[col * width + row];
}

PointCloudTransform::PointCloudTransform(uint32_t left, uint32_t top, uint32_t width, uint32_t height, 
                                         uint32_t rowsToMerge, uint32_t columnsToMerge,
                                         float fx, float fy, float cx, float cy,
                                         float k1, float k2, float k3, float p1, float p2)
{
  this->left = left;
  this->top = top;
  this->width = width;
  this->height = height;
  this->rowsToMerge = rowsToMerge;
  this->columnsToMerge = columnsToMerge;
  this->fx = fx;
  this->fy = fy;
  this->cx = cx;
  this->cy = cy;
  this->k1 = k1;
  this->k2 = k2;
  this->k3 = k3;
  this->p1 = p1;
  this->p2 = p2;
  directions.clear();
  _init();
}

void PointCloudTransform::calcAperatureAngleRadians(int &horizontalAperatureAngle, float &verticalAperatureAngle)
{
  Point &p1 = getDirection((int)cy, 0);

  horizontalAperatureAngle = getDirection((int)cy, 0).angle(getDirection((int)cy, width - 1));
  verticalAperatureAngle = getDirection(0, (int)cx).angle(getDirection(height - 1, (int)cx));
}

void PointCloudTransform::rescaleParameters(int width, int height)
{
  float scaleFx = (float)width / (float)this->width;
  float scaleFy = (float)height / (float)this->height;
  float scaleCx = (float)(width - 1) / (float)(this->width - 1);
  float scaleCy = (float)(height - 1) / (float)(this->height - 1);

  this->height = height;
  this->width = width;
  this->fx *= scaleFx;
  this->fy *= scaleFy;
  this->cx *= scaleCx;
  this->cy *= scaleCy;
  this->directions.clear();
}

// Private methods

Point PointCloudTransform::_screenToNormalizedScreen(const Point &screen, bool verify)
{
  int iters = 100;
  float xs, ys;
  float yss = ys = (screen.y - cy) / fy;
  float xss = xs = (screen.x - cx) / fx;

  for(int j = 0; j < iters; j++)
  {
    float r2 = xs * xs + ys * ys;
    float icdist = 1.0f / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
    float deltaX = 2 * p1 * xs * ys + p2 * (r2 + 2 * xs * xs);
    float deltaY = p1 * (r2 + 2 * ys * ys) + 2 * p2 * xs * ys;
    xs = (xss - deltaX)*icdist;
    ys = (yss - deltaY)*icdist;
  }

  if(verify)
  {
    float x_ = xs;
    float y_ = ys;
    float r2 = x_ * x_ + y_ * y_;
    float r4 = r2 * r2;
    float r6 = r2 * r4;

    float x__ = x_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + 2.0f * p1 * x_ * y_ + p2 * (r2 + 2.0f * x_ * x_);
    float y__ = y_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2.0f * y_ * y_) + 2.0f * p2 * x_ * y_;

    if(fabs(x__ - xss) > FLOAT_EPSILON || fabs(y__ - yss) > FLOAT_EPSILON)
    {
      return Point(POINT_INVALID, 0);
    }
    else
    {
      return Point(xs, ys);
    }
  }
  else
  {
    return Point(xs, ys);
  }
}

Point PointCloudTransform::_normalizedScreenToScreen(const Point &normalizedScreen)
{
  float x_ = normalizedScreen.x;
  float y_ = normalizedScreen.y;
  float r2 = x_ * x_ + y_ * y_;
  float r4 = r2 * r2;
  float r6 = r2 * r4;

  float x__ = x_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + 2.0f * p1 * x_ * y_ + p2 * (r2 + 2.0f * x_ * x_);
  float y__ = y_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2.0f * y_ * y_) + 2.0f * p2 * x_ * y_;

  return (Point(x__, y__));
}

Point PointCloudTransform::_lensCorrection(const Point &normalizedScreen)
{
  float x_ = (normalizedScreen.x - cx)/fx;
  float y_ = (normalizedScreen.y - cy)/fy;
  float r2 = x_ * x_ + y_ * y_;
  float r4 = r2 * r2;
  float r6 = r2 * r4;
  
  float x__ = x_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + 2.0f * p1 * x_ * y_ + p2 * (r2 + 2.0f * x_ * x_);
  float y__ = y_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2.0f * y_ * y_) + 2.0f * p2 * x_ * y_;
  
  return (Point(x__, y__));
}



bool PointCloudTransform::_computeConcaveMirrorBorders(int width, int height, 
                                                       double &minX, double &maxX,
                                                       double &minY, double &maxY)
{
  Vector<double> leftArr;
  Vector<double> rightArr;
  Vector<double> topArr;
  Vector<double> bottomArr;

  _computeConcaveMirror(width, height, leftArr, rightArr, topArr, bottomArr);
  
  if(leftArr.size() == 0 || rightArr.size() == 0 || topArr.size() == 0 || bottomArr.size() == 0)
  {
    return false;
  }

  minX = *std::max_element(leftArr.begin(), leftArr.end());
  maxX = *std::min_element(rightArr.begin(), rightArr.end());
  minY = *std::max_element(topArr.begin(), topArr.end());
  maxY = *std::min_element(bottomArr.begin(), bottomArr.end());
  
  return true;
}

bool PointCloudTransform::_computeConcaveMirrorBordersOuter(int width, int height, double &minX, double &maxX,
    double &minY, double &maxY)
{
  Vector<double> leftArr;
  Vector<double> rightArr;
  Vector<double> topArr;
  Vector<double> bottomArr;

  _computeConcaveMirror(width, height, leftArr, rightArr, topArr, bottomArr);
  if(leftArr.size() == 0 || rightArr.size() == 0 || topArr.size() == 0 || bottomArr.size() == 0)
  {
    return false;
  }

  minX = *std::min_element(leftArr.begin(), leftArr.end());
  maxX = *std::max_element(rightArr.begin(), rightArr.end());
  minY = *std::min_element(topArr.begin(), topArr.end());
  maxY = *std::max_element(bottomArr.begin(), bottomArr.end());

  return true;
}

void PointCloudTransform::_computeConcaveMirror(int width, int height, Vector<double> &leftArr, Vector<double> &rightArr,
    Vector<double> &topArr, Vector<double> &bottomArr)
{
  leftArr.clear();
  rightArr.clear();
  topArr.clear();
  bottomArr.clear();

  int xCountTop = 0;
  int xCountBottom = 0;
  for(int x = 0; x < width; x++)
  {
    Point normedScreenCoords = _screenToNormalizedScreen(Point(x, 0.0f), true);
    if(normedScreenCoords.x != POINT_INVALID)
    {
      topArr.push_back(normedScreenCoords.y);
      xCountTop++;
    }
    
    normedScreenCoords = _screenToNormalizedScreen(Point(x, height - 1), true);
    if(normedScreenCoords.x != POINT_INVALID)
    {
      bottomArr.push_back(normedScreenCoords.y);
      xCountBottom++;
    }
  }

  int yCountLeft = 0;
  int yCountRight = 0;
  for(int y = 0; y < height; y++)
  {
    Point normedScreenCoords = _screenToNormalizedScreen(Point(0, y), true);
    if(normedScreenCoords.x != POINT_INVALID)
    {
      leftArr.push_back(normedScreenCoords.x);
    }
    normedScreenCoords = _screenToNormalizedScreen(Point(width - 1, y), true);
    if(normedScreenCoords.x != POINT_INVALID)
    {
      rightArr.push_back(normedScreenCoords.x);
    }
  }
}

void PointCloudTransform::_init()
{
  directions.clear();
  for(int v = top; v < top + height; v++)
  {
    for(int u = left; u < left + width; u++)
    {
      Point normalizedScreen = _screenToNormalizedScreen(Point(u, v), true);
      //Point normalizedScreen = _lensCorrection(Point(u, v));
      Point dir = _normalizedScreenToUnitWorld(normalizedScreen);
      directions.push_back(dir);
    }
  }
  _computeClippingPlanes();
}

Point PointCloudTransform::_normalizedScreenToUnitWorld(const Point &normalizedScreen)
{
  float _norm = 1.0f / (float)sqrt(normalizedScreen.x * normalizedScreen.x
                                   + normalizedScreen.y * normalizedScreen.y
                                   + 1.0f);
  return Point(normalizedScreen.x * _norm, normalizedScreen.y * _norm, _norm);
}

void PointCloudTransform::_computeClippingPlanes()
{
  float extension = (float)width;
  Point topLeftNormalized;
  Point topRightNormalized;
  Point bottomLeftNormalized;
  Point bottomRightNormlized;
  float ext;
  for(ext = extension; ext > 0.0f; ext -= 1.0f)
  {
    topLeftNormalized = _screenToNormalizedScreen(Point(left - ext, top - ext), true);
    if(topLeftNormalized.x == POINT_INVALID) { continue; }
    topRightNormalized = _screenToNormalizedScreen(Point((float)width + left + ext, top - ext), true);
    if(topRightNormalized.x == POINT_INVALID) { continue; }
    bottomLeftNormalized = _screenToNormalizedScreen(Point(left - ext, (float)height + top + ext), true);
    if(bottomLeftNormalized.x == POINT_INVALID) { continue; }
    bottomRightNormlized = _screenToNormalizedScreen(Point((float)width + left + ext,
                           (float)height + top + ext), true);
    if(bottomRightNormlized.x == POINT_INVALID) { continue; }
    break;
  }

  Point topLeft, topRight, bottomLeft, bottomRight;
  if(ext == 0.0f)
  {
    topLeft = Point(-1000.0f, -1000.0f, 1000.0f);
    topRight = Point(1000.0f, -1000.0f, 1000.0f);
    bottomLeft = Point(-1000.0f, 1000.0f, 1000.0f);
    bottomRight = Point(1000.0f, 1000.0f, 1000.0f);
  }
  else
  {
    topLeft = _normalizedScreenToUnitWorld(topLeftNormalized);
    topRight = _normalizedScreenToUnitWorld(topRightNormalized);
    bottomLeft = _normalizedScreenToUnitWorld(bottomLeftNormalized);
    bottomRight = _normalizedScreenToUnitWorld(bottomRightNormlized);
  }

  // FIXME: In original implementation, ofPoint's cross() modifies topLeft and topRight
  leftClippingNormal = topLeft.cross(bottomLeft);
  rightClippingNormal = topRight.cross(bottomRight);
  if(leftClippingNormal.x * rightClippingNormal.x < 0)   // let normal point to the same direction
  {
    rightClippingNormal = -rightClippingNormal;
  }

  // FIXME: In original implementation, topLeft is not the same as the original as cross() modifie
  topClippingNormal = topLeft.cross(topRight);
  bottomClippingNormal = bottomLeft.cross(bottomRight);
  if(bottomClippingNormal.y * topClippingNormal.y < 0)    // let normal point to the same direction
  {
    bottomClippingNormal = -bottomClippingNormal;
  }
}

Point PointCloudTransform::worldToImage(const Point &p)
{
  //test if point is lying in view frustrum
  float l = p.dot(leftClippingNormal);  // distance to plane
  float r = p.dot(rightClippingNormal);
  if(l * r > 0.0f) { return Point(POINT_INVALID, POINT_INVALID); } // outside frustrum
  float t = p.dot(topClippingNormal);
  float b = p.dot(bottomClippingNormal);
  if(t * b > 0.0f) { return Point(POINT_INVALID, POINT_INVALID); } // outside frustrom

  float x_ = p.x / p.z;
  float y_ = p.y / p.z;

  float r2 = x_ * x_ + y_ * y_;
  float r4 = r2 * r2;
  float r6 = r2 * r4;

  float xss = x_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + 2.0f * p1 * x_ * y_ + p2 * (r2 + 2.0f * x_ * x_);
  float yss = y_ * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2.0f * y_ * y_) + 2.0f * p2 * x_ * y_;

  return Point((float)fx * xss + (float)cx - left, (float)fy * yss + (float)cy - top);
}

Point PointCloudTransform::imageToWorld(const Point &p, float depth)
{
  if(fabs(p.x - (int)p.x) < FLOAT_EPSILON && fabs(p.y - (int)p.y) < FLOAT_EPSILON && 
    p.y <= (float)height - 0.51f && p.x <= (float)width - 0.51f && 
    p.x >= -0.49f && p.y >= -0.49f)
  {
    return getDirection((int)(p.y + 0.5), (int)(p.x + 0.5)) * depth;
  }
  else
  {
    // for subpoixel accuracy
    if(fx < 0.0001 || fy < 0.0001)
    {
      return Point(POINT_INVALID, POINT_INVALID);
    }
    Point normalizedScreen = _screenToNormalizedScreen(p + Point(left, top), false);
    return _normalizedScreenToUnitWorld(normalizedScreen) * depth;
  }
}

bool PointCloudTransform::depthToPointCloud(const Vector<float> &distances, PointCloudFrame &pointCloudFrame)
{
#ifdef COMMON_OPT
  uint w, h;
  if((columnsToMerge == 1) && (rowsToMerge == 1))
    w = width, h = height;
  else
    w = (width + columnsToMerge - 1)/columnsToMerge, h = (height + rowsToMerge - 1)/rowsToMerge;
   
  auto nFrameSize = w*h;
  if(distances.size() < nFrameSize ||
    pointCloudFrame.size() < nFrameSize)
    return false;

  auto mergedWidth = width/columnsToMerge;

  if(columnsToMerge == 1)
  {
    for(int v = 0; v < height; v += rowsToMerge)
    {
      int v_merged = v/rowsToMerge *  mergedWidth;  
      int v_width = v * width;
#ifdef x86_OPT
      for(int u = 0; u < width; u += 4)
      {
        int idx = v_width + u;
        int idx_1 = v_width + u + 1;
        int idx_2 = v_width + u + 2;
        int idx_3 = v_width + u + 3;

        int idx2 = v_merged + u;
        int idx3 = v_merged + u + 1;
        int idx4 = v_merged + u + 2;
        int idx5 = v_merged + u + 3;

        Point *p = pointCloudFrame[idx2];
        Point *p1 = pointCloudFrame[idx3];
        Point *p2 = pointCloudFrame[idx4];
        Point *p3 = pointCloudFrame[idx5];

        *p = directions[idx] * distances[idx2];
        *p1 = directions[idx_1] * distances[idx3];
        *p2 = directions[idx_2] * distances[idx4];
        *p3 = directions[idx_3] * distances[idx5];
      }
#elif ARM_OPT
      int widthm4 = width - 4;
      int u;

      for(u = 0; u < width; u += 4)
      {
        int idx  = v_width + u;
        int idx2 = v_merged + u;

        Point *p = pointCloudFrame[idx2];

        float32x4_t ndist;
        float32x4x4_t nPC;
        float32x4x3_t nDir;
        
        nDir = vld3q_f32((float32_t *)&directions[idx]);
        ndist = vld1q_f32((float32_t *)&distances[idx2]);
        
        nPC.val[0] = vmulq_f32(nDir.val[0], ndist);
        nPC.val[1] = vmulq_f32(nDir.val[1], ndist);
        nPC.val[2] = vmulq_f32(nDir.val[2], ndist);
        
        vst4q_f32((float32_t *)(&p->x), nPC);

      }
#endif
    }
  }
  else
  {
    for(int v = 0; v < height; v += rowsToMerge)
    { 
      int v_merged = v/rowsToMerge *  mergedWidth;  
      int v_width = v * width;
      for(int u = 0; u < width; u += columnsToMerge)
      {
        int idx = v_width + u;
        int idx2 = v_merged + u/columnsToMerge;
        Point *p = pointCloudFrame[idx2];

        if(p)
          *p = directions[idx] * distances[idx2];
      }
    }
  }
#else
  uint w = (width + columnsToMerge - 1)/columnsToMerge, h = (height + rowsToMerge - 1)/rowsToMerge;

  if(distances.size() < w*h ||
    pointCloudFrame.size() < w*h)
    return false;

  for(int v = 0; v < height; v += rowsToMerge)
  {
    for(int u = 0; u < width; u += columnsToMerge)
    {
      int idx = v * width + u;
      int idx2 = v/rowsToMerge * width/columnsToMerge + u/columnsToMerge;
      Point *p = pointCloudFrame[idx2];

      if(p)
        *p = directions[idx] * distances[idx2];
//       else
//       {
//         logger(LOG_ERROR) << "PointCloudTransform: Could not set point at (" << u << ", " << v << ")" << std::endl;
//         return false;
//       }
    }
  }
#endif
  return true;
}

}
