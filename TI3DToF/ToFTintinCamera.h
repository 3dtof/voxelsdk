/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFTINTINCAMERA_H
#define VOXEL_TI_TOFTINTINCAMERA_H

#include <ToFCamera.h>

#define MOD_M1 "mod_m1" //mdiv_fb_deci_pll1
#define MOD_M_FRAC1 "mod_m_frac1"
#undef MOD_M2
#define MOD_M2 "mod_m2" //mdiv_fb_deci_pll2
#define MOD_M_FRAC2 "mod_m_frac2"
#define MOD_N1 "mod_n1" //mdiv_pll_1_in
#undef MOD_N2
#define MOD_N2 "mod_n2" //mdiv_pll_2_in

#define VCO_FREQ1 "vco_freq1"
#define VCO_FREQ2 "vco_freq2"

#undef MOD_PS1
#define MOD_PS1 "mod_ps1" //mdiv_ff_pll1
#undef MOD_PS2
#define MOD_PS2 "mod_ps2" //mdiv_ff_pll2
#undef MOD_PLL_UPDATE
#define MOD_PLL_UPDATE "mod_pll_update" //reset_pll_reg

#undef DEBUG_EN
#define DEBUG_EN "phy_test_enable"

#undef CONFIDENCE_THRESHOLD
#define CONFIDENCE_THRESHOLD "amplitude_threshold"

#define KA "ka"
#define KB "kb"
#define FREQ_RATIO "freq_ratio"
#define DELAY_FB_COEFF1 "delay_fb_coeff_1"

#undef HISTOGRAM_EN

#define TG_DISABLE "tg_dis"

#define SCRATCH1 "scratch1" // Used for storing unambiguous range
#define SCRATCH2 "scratch2" // Used for storing current camera profile ID

#define X_CROSS_TALK_COEFF_F1 "x_cross_talk_coeff_f1"
#define Y_CROSS_TALK_COEFF_F1 "y_cross_talk_coeff_f1"
#define X_CROSS_TALK_COEFF_F2 "x_cross_talk_coeff_f2"
#define Y_CROSS_TALK_COEFF_F2 "y_cross_talk_coeff_f2"

#define CROSS_TALK_FILT_COEFF_X_RE_F1 "filt_coeff_x_re_f1"
#define CROSS_TALK_FILT_COEFF_X_IM_F1 "filt_coeff_x_im_f1"
#define CROSS_TALK_FILT_COEFF_Y_RE_F1 "filt_coeff_y_re_f1"
#define CROSS_TALK_FILT_COEFF_Y_IM_F1 "filt_coeff_y_im_f1"
#define CROSS_TALK_FILT_COEFF_X_RE_F2 "filt_coeff_x_re_f2"
#define CROSS_TALK_FILT_COEFF_X_IM_F2 "filt_coeff_x_im_f2"
#define CROSS_TALK_FILT_COEFF_Y_RE_F2 "filt_coeff_y_re_f2"
#define CROSS_TALK_FILT_COEFF_Y_IM_F2 "filt_coeff_y_im_f2"
#define CROSS_TALK_EN "filt_en"
#define CROSS_TALK_SCALE "filt_scale"

#define NONLINEARITY_PHASE_PERIOD "phase_lin_corr_period"
#define NONLINEARITY_COEFF_F1 "phase_lin_coeff0"
#define NONLINEARITY_COEFF_F2 "phase_lin_coeff1"
#define NONLINEARITY_ENABLE "phase_lin_corr_en"

#define COEFF_ILLUM_1 "coeff_illum"
#define COEFF_SENSOR_1 "coeff_sensor"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFTintinCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  virtual bool _getIlluminationFrequency(float& frequency) const;
  
  virtual bool _getSystemClockFrequency(uint &frequency) const;
  virtual bool _allowedROI(String &message);
  virtual bool _getROI(RegionOfInterest &roi);
  virtual bool _setROI(const RegionOfInterest &roi);
  
  virtual bool _isHistogramEnabled() const;
  
  virtual bool _applyCalibrationParams();
  
  virtual bool _getCurrentProfileRegisterName(String& name);
    
public:
  ToFTintinCamera(const String &name, DevicePtr device);
  
  virtual bool _getMaximumFrameSize(FrameSize &s) const;
  virtual ~ToFTintinCamera() {}
  
  friend class TintinVCOFrequency;
  friend class TintinModulationFrequencyParameter;
  friend class TintinUnambiguousRangeParameter;
};

}
}

#endif // VOXEL_TI_TOFTINTINCAMERA_H
