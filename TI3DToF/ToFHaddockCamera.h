/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFHADDOCKCAMERA_H
#define VOXEL_TI_TOFHADDOCKCAMERA_H

#include <ToFCamera.h>

#define PIX_CNT_MAX_SET_FAILED "pix_cnt_max_set_failed"
#define PIX_CNT_MAX "pix_cnt_max"
#define QUAD_CNT_MAX "quad_cnt_max"
#define SUBFRAME_CNT_MAX "uframe_cnt_max"
#define SYS_CLK_FREQ "sys_clk_freq"

#define TG_EN "tg_en"
#define BLK_SIZE "blk_size"
#define BLK_HEADER_EN "blk_header_en"
#define OP_CS_POL "op_cs_pol"
#define FB_READY_EN "fb_ready_en"

#define PIXEL_DATA_SIZE "pixel_data_size"
#define OP_DATA_ARRANGE_MODE "op_data_arrange_mode"
#define HISTOGRAM_EN "histogram_en"

namespace Voxel
{
  
namespace TI
{

class ToFHaddockCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  virtual bool _processRawFrame(RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput); // here output raw frame will have processed data, like ToF data for ToF cameras
  
public:
  ToFHaddockCamera(const String &name, DevicePtr device);
  
  virtual bool setFrameRate(const FrameRate &r);
  virtual bool getFrameRate(FrameRate &r);
  
  virtual bool getFrameSize(FrameSize &s);
  virtual bool setFrameSize(const FrameSize &s);
  
  virtual ~ToFHaddockCamera() {}
};

}
}

#endif // VOXEL_TI_TOFHADDOCKCAMERA_H
