/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFCALCULUSCAMERA_H
#define VOXEL_TI_TOFCALCULUSCAMERA_H

#include <ToFCamera.h>

#undef MOD_M
#define MOD_M "mod_m1"
#define MOD_M1 "mod_m1"
#define MOD_M2 "mdiv_fb_deci_pll2"
#undef MOD_N
#define MOD_N "mod_n1"
#define MOD_N1 "mod_n1"
#define MOD_N2 "mdiv_pll_2_in"

#undef MOD_PS1
#define MOD_PS1 "mod_ps1"
#undef MOD_PS2
#define MOD_PS2 "mdiv_ff_pll2"
#undef MOD_PLL_UPDATE
#define MOD_PLL_UPDATE "mod_pll_update"
#undef ROW_START
#define ROW_START "row_addr1_start"
#undef ROW_END
#define ROW_END "row_addr1_end"
#undef COL_START
#define COL_START "col_addr1_start"
#undef COL_END
#define COL_END "col_addr1_end"
#undef QUAD_CNT_MAX
#define QUAD_CNT_MAX "tm_quad_cnt_max"

#define EN_GLOBAL_PDN_REG "en_global_pdn_reg"
#define DIS_SDMOD "dis_sdmod"
#define ADCK_R "adck_r"
#define UPDATE_SEL "update_sel"
#define SYNC_INTEG_PHASE_EN "sync_integ_phase_en"
#define TM_INTG_PHASE_PIX1 "tm_intg_phase_pix1"
#define DVP_BLANK_PIX_PER_LINE "dvp_blank_pix_per_line"
#define DISABLE_REARRANGE "disable_rearrange"
#define TM_PDZ_LED_DRIVER "enable_led_driver"
#define LED_DRIVER_CURR_PROG "led_driver_curr_prog"
#define MCKADGEN_R_OFFSET "mckadgen_r_offset"
#define SHUTTER_EN "shutter_en"
#define TEST_PG1 "test_pg1"
#define TM_PIX_CNT_MAX "pix_cnt_max"
#define TM_UFRAME_CNT_MAX "sub_frame_cnt_max"
#define INTEGRATION_DUTY_CYCLE "intg_duty_cycle"
#define FLIP_PHASE "flip_phase"
#define CALIB_PHASE_OFFSET "phase_corr_1"

#undef CONFIDENCE_THRESHOLD
#define CONFIDENCE_THRESHOLD "confidence_threshold"
#define CLIP_MODE "clip_mode"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFCalculusCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  virtual bool _getDepthScalingFactor(float &factor);
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
  
  virtual bool _isHistogramEnabled() const;
  
  virtual bool _applyCalibrationParams();

  bool _updateEasyConf();
    
public:
  ToFCalculusCamera(const String &name, DevicePtr device);
  
  
  
  virtual ~ToFCalculusCamera() {}
  
  friend class CalculusVCOFrequency;
  friend class CalculusModulationFrequencyParameter;
  friend class CalculusIntegDutyCycle;
  friend class CalculusPixCntMax;
  friend class CalculusSubFrameCntMax;
};

}
}

#endif // VOXEL_TI_TOFCALCULUSCAMERA_H
