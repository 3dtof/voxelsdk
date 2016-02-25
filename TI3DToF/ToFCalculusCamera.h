/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFCALCULUSCAMERA_H
#define VOXEL_TI_TOFCALCULUSCAMERA_H

#include <ToFCamera.h>

#undef MOD_M
#define MOD_M "mod_m"
#define MOD_M1 "mod_m"
#define MOD_M_FRAC1 "mod_m_frac"
#define MOD_M2 "mdiv_fb_deci_pll2"
#undef MOD_N
#define MOD_N "mod_n"
#define MOD_N1 "mod_n"
#define MOD_N2 "mdiv_pll_2_in"

#undef MOD_PS1
#define MOD_PS1 "mod_ps"
#undef MOD_PS2
#define MOD_PS2 "mdiv_ff_pll2"
#undef MOD_PLL_UPDATE
#define MOD_PLL_UPDATE "mod_pll_update"
#undef ROW_START
#define ROW_START "row_start"
#undef ROW_END
#define ROW_END "row_end"
#undef COL_START
#define COL_START "col_start"
#undef COL_END
#define COL_END "col_end"
#undef QUAD_CNT_MAX
#define QUAD_CNT_MAX "quad_cnt_max"

#define STANDBY "standby"
#define DIS_SDMOD "dis_sdmod"
#define INIT_0 "init_0"
#define INIT_1 "init_1"
#define INIT_2 "init_2"
#define INIT_3 "init_3"
#define UPDATE_SEL "update_sel"
#define OP_DATA_ARRANGE_MODE "op_data_arrange_mode"
#define MOD_CDRIV_EN "mod_cdriv_en"
#define MOD_CDRIV_CURR "mod_cdriv_curr"
#define SHUTTER_EN "shutter_en"
#define INTEGRATION_DUTY_CYCLE "intg_duty_cycle"

#undef CONFIDENCE_THRESHOLD
#define CONFIDENCE_THRESHOLD "amplitude_threshold"

#undef SUBFRAME_CNT_MAX
#define SUBFRAME_CNT_MAX "sub_frame_cnt_max1"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFCalculusCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  virtual bool _getIlluminationFrequency(float& frequency) const;
  bool _getMaximumFrameSize(FrameSize &s) const;
  
  virtual bool _getFrameSize(Voxel::FrameSize &s) const;
  virtual bool _getBytesPerPixel(uint &bpp) const;
  virtual bool _getOpDataArrangeMode(int &dataArrangeMode) const;
  virtual bool _getFrameRate(FrameRate &r) const;
  virtual bool _setFrameRate(const FrameRate &r);
  virtual bool _getSystemClockFrequency(uint &frequency) const;
  
  
  
  virtual bool _getToFFrameType(ToFFrameType &frameType) const;
  virtual bool _allowedROI(String &message);
  virtual bool _getROI(RegionOfInterest &roi);
  virtual bool _setROI(const RegionOfInterest &roi);
  virtual bool _getBinning(uint &rowsToMerge, uint &columnsToMerge) const;
  virtual bool _setBinning(uint rowsToMerge, uint columnsToMerge, const FrameSize &frameSize);
  
  virtual bool _is16BitModeEnabled(bool &mode16Bit);
  virtual bool _getDealiasedPhaseMask(int &dealiasedPhaseMask);
  
  virtual bool _getSubFrameCount(int &subframeCount) const;
  
  virtual bool _isHistogramEnabled() const;
  
  virtual bool _applyCalibrationParams();

  bool _updateEasyConf();
  
  virtual bool _getCurrentProfileRegisterName(String& name);
    
public:
  ToFCalculusCamera(const String &name, DevicePtr device);
  
  
  
  virtual ~ToFCalculusCamera() {}
  
  friend class CalculusVCOFrequency;
  friend class CalculusModulationFrequencyParameter;
  friend class CalculusIntegDutyCycle;
};

}
}

#endif // VOXEL_TI_TOFCALCULUSCAMERA_H
