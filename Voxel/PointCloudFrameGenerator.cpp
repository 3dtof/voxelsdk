/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <PointCloudFrameGenerator.h>

#include <DepthCamera.h>

#define PARAM_ROI_X "roiX"
#define PARAM_ROI_Y "roiY"
#define PARAM_ROI_WIDTH "roiWidth"
#define PARAM_ROI_HEIGHT "roiHeight"
#define PARAM_ROWS_TO_MERGE "rowsToMerge"
#define PARAM_COLUMNS_TO_MERGE "columnsToMerge"
#define PARAM_CX "cx"
#define PARAM_CY "cy"
#define PARAM_FX "fx"
#define PARAM_FY "fy"
#define PARAM_K1 "k1"
#define PARAM_K2 "k2"
#define PARAM_K3 "k3"
#define PARAM_P1 "p1"
#define PARAM_P2 "p2"

namespace Voxel
{

PointCloudFrameGenerator::PointCloudFrameGenerator():
  FrameGenerator(0, DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, 0, 1) 
{
  _frameGeneratorParameters[PARAM_ROI_X] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROI_Y] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROI_WIDTH] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROI_HEIGHT] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROWS_TO_MERGE] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_COLUMNS_TO_MERGE] = SerializablePtr(new SerializableUnsignedInt());
  
  _frameGeneratorParameters[PARAM_CX] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_CY] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_FX] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_FY] = SerializablePtr(new SerializableFloat());
  
  _frameGeneratorParameters[PARAM_K1] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_K2] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_K3] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_P1] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_P2] = SerializablePtr(new SerializableFloat());
}

bool PointCloudFrameGenerator::setParameters(uint32_t left, uint32_t top, uint32_t width, uint32_t height, 
                                             uint32_t rowsToMerge, uint32_t columnsToMerge,
                                             float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2)
{
  if(_pointCloudTransform)
  {
    if(_pointCloudTransform->left == left && _pointCloudTransform->top == top &&
      _pointCloudTransform->height == height && _pointCloudTransform->width == width &&
      _pointCloudTransform->rowsToMerge == rowsToMerge && _pointCloudTransform->columnsToMerge == columnsToMerge &&
      _pointCloudTransform->fx == fx && _pointCloudTransform->fy == fy &&
      _pointCloudTransform->cx == cx && _pointCloudTransform->cy == cy &&
      _pointCloudTransform->k1 == k1 && _pointCloudTransform->k2 == k2 && _pointCloudTransform->k3 == k3 &&
      _pointCloudTransform->p1 == p1 && _pointCloudTransform->p2 == p2)
      return true; // No need to change anything
  }
  
  if(left < 0 || top < 0 || height < 0 || width < 0 || rowsToMerge < 0 || columnsToMerge < 0)
  {
    logger(LOG_ERROR) << "PointCloudFrameGenerator: Parameters values are invalid. "
      << "left = " << left << ", "
      << "top = " << top << ", "
      << "width = " << width << ", "
      << "height = " << height << ", "
      << "rowsToMerge = " << rowsToMerge << ", "
      << "columnsToMerge = " << columnsToMerge << std::endl;
    return false;
  }
  
  _pointCloudTransform = Ptr<PointCloudTransform>(new PointCloudTransform(left, top, width, height, rowsToMerge, columnsToMerge, fx, fy, cx, cy, k1, k2, k3, p1, p2));
  
  if(
    !_set(PARAM_ROI_X, left) ||
    !_set(PARAM_ROI_Y, top) ||
    !_set(PARAM_ROI_WIDTH, width) ||
    !_set(PARAM_ROI_HEIGHT, height) ||
    !_set(PARAM_ROWS_TO_MERGE, rowsToMerge) ||
    !_set(PARAM_COLUMNS_TO_MERGE, columnsToMerge) ||
    !_set(PARAM_CX, cx) ||
    !_set(PARAM_CY, cy) ||
    !_set(PARAM_FX, fx) ||
    !_set(PARAM_FY, fy) ||
    !_set(PARAM_K1, k1) ||
    !_set(PARAM_K2, k2) ||
    !_set(PARAM_K3, k3) ||
    !_set(PARAM_P1, p1) ||
    !_set(PARAM_P2, p2))
    return false;
  
  return writeConfiguration();
}

bool PointCloudFrameGenerator::_onWriteConfiguration()
{
  if(!_pointCloudTransform)
    return false;
  return true;
}

bool PointCloudFrameGenerator::_onReadConfiguration()
{
  uint32_t left, top, width, height, rowsToMerge, columnsToMerge;
  float fx, fy, cx, cy, k1, k2, k3, p1, p2;
  
  if(
    !get(PARAM_ROI_X, left) ||
    !get(PARAM_ROI_Y, top) ||
    !get(PARAM_ROI_WIDTH, width) ||
    !get(PARAM_ROI_HEIGHT, height) ||
    !get(PARAM_ROWS_TO_MERGE, rowsToMerge) ||
    !get(PARAM_COLUMNS_TO_MERGE, columnsToMerge) ||
    !get(PARAM_CX, cx) ||
    !get(PARAM_CY, cy) ||
    !get(PARAM_FX, fx) ||
    !get(PARAM_FY, fy) ||
    !get(PARAM_K1, k1) ||
    !get(PARAM_K2, k2) ||
    !get(PARAM_K3, k3) ||
    !get(PARAM_P1, p1) ||
    !get(PARAM_P2, p2))
    return false;
    
  if(left < 0 || top < 0 || height < 0 || width < 0 || rowsToMerge < 0 || columnsToMerge < 0)
  {
    logger(LOG_ERROR) << "PointCloudFrameGenerator: Parameters values are invalid. "
    << "left = " << left << ", "
    << "top = " << top << ", "
    << "width = " << width << ", "
    << "height = " << height << ", "
    << "rowsToMerge = " << rowsToMerge << ", "
    << "columnsToMerge = " << columnsToMerge << std::endl;
    return false;
  }
  
  _pointCloudTransform = Ptr<PointCloudTransform>(new PointCloudTransform(left, top, width, height, rowsToMerge, columnsToMerge, fx, fy, cx, cy, k1, k2, k3, p1, p2));    
  return true;
}

bool PointCloudFrameGenerator::generate(const FramePtr &in, FramePtr &out)
{
  const DepthFrame *depthFrame = dynamic_cast<const DepthFrame *>(in.get());
  
  if(!depthFrame)
  {
    logger(LOG_ERROR) << "PointCloudFrameGenerator: Only DepthFrame type is supported as input to generate() function." << std::endl;
    return false;
  }
  
  XYZIPointCloudFrame *f = dynamic_cast<XYZIPointCloudFrame *>(out.get());
  
  if(!f)
  {
    f = new XYZIPointCloudFrame();
    out = FramePtr(f);
  }
  
  f->id = depthFrame->id;
  f->timestamp = depthFrame->timestamp;
  f->points.resize(depthFrame->size.width*depthFrame->size.height);
  
  if(!_pointCloudTransform->depthToPointCloud(depthFrame->depth, *f))
  {
    logger(LOG_ERROR) << "DepthCamera: Could not convert depth frame to point cloud frame" << std::endl;
    return false;
  }
  
  // Setting amplitude as intensity
  auto index = 0;
  
  auto w = depthFrame->size.width;
  auto h = depthFrame->size.height;
  
  for(auto y = 0; y < h; y++)
    for(auto x = 0; x < w; x++, index++)
    {
      IntensityPoint &p = f->points[index];
      p.i = depthFrame->amplitude[index];
    }
    
  return true;
}

  
  
}