/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFHADDOCKCAMERA_H
#define VOXEL_TI_TOFHADDOCKCAMERA_H

#include <ToFCamera.h>

#include "TI3DToFExports.h"

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

#define CONFIDENCE_THRESHOLD "confidence_threshold"

#define ILLUM_EN_POL "illum_en_pol"

#define BIN_ROWS_TO_MERGE "rows_to_merge"
#define BIN_COLS_TO_MERGE "cols_to_merge"
#define BIN_ROW_COUNT "bin_row_count"
#define BIN_COLUMN_COUNT "bin_col_count"
#define BINNING_EN "binning_en"

#define ROW_START "row_addr1_start"
#define ROW_END "row_addr1_end"
#define COL_START "col_addr1_start"
#define COL_END "col_addr1_end"

#define DEBUG_EN "debug_en"

#define PIXEL_DATA_SIZE "pixel_data_size"
#define OP_DATA_ARRANGE_MODE "op_data_arrange_mode"
#define HISTOGRAM_EN "histogram_en"

#define INTG_TIME "intg_time"  // Integration time
#define INTG_DUTY_CYCLE "intg_duty_cycle"
#define INTG_DUTY_CYCLE_SET_FAILED "intg_duty_cycle_set_failed"
#define MOD_FREQ1 "mod_freq1" // Modulation frequency for first source (MHz)
#define MOD_FREQ2 "mod_freq2" // Modulation frequency for second source (MHz)
#define VCO_FREQ "vco_freq"
#define DEALIAS_EN "dealias_en"
#define MOD_PS1 "mod_ps1"
#define MOD_PS2 "mod_ps2"
#define MOD_M "mod_m"
#define MOD_N "mod_n"
#define MOD_PLL_UPDATE "mod_pll_update"

#define SPEED_OF_LIGHT 3E8f

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFHaddockCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  virtual bool _processRawFrame(const RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput); // here output raw frame will have processed data, like ToF data for ToF cameras
  
  virtual bool _getAmplitudeNormalizingFactor(float &factor);
  virtual bool _getDepthScalingFactor(float &factor);
  
  virtual bool _setFrameRate(const FrameRate &r);
  virtual bool _getFrameRate(FrameRate &r) const;
  
  virtual bool _getFrameSize(FrameSize &s) const;
  virtual bool _getMaximumFrameSize(FrameSize &s) const;
  virtual bool _setFrameSize(const FrameSize &s);
  virtual bool _setFrameSize(const FrameSize &s, bool resetROI);
  virtual bool _setStreamerFrameSize(const FrameSize &s) = 0;
  
  virtual bool _setBinning(uint rowsToMerge, uint columnsToMerge, VideoMode &videoMode);
  
  virtual bool _allowedROI(String &message);
  virtual bool _getROI(RegionOfInterest &roi);
  virtual bool _setROI(const RegionOfInterest &roi);
  
public:
  ToFHaddockCamera(const String &name, DevicePtr device);
  
  virtual ~ToFHaddockCamera() {}
  
  friend class VCOFrequency;
  friend class ModulationFrequencyParameter;
  friend class IntegrationTimeParameter;
};

}
}

#endif // VOXEL_TI_TOFHADDOCKCAMERA_H
