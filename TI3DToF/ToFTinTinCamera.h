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

#undef HISTOGRAM_EN

#define TG_DISABLE "tg_dis"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFTinTinCamera: public ToFCamera
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
    
public:
  ToFTinTinCamera(const String &name, DevicePtr device);
  
  virtual bool _getMaximumFrameSize(FrameSize &s) const;
  virtual ~ToFTinTinCamera() {}
  
  friend class TinTinVCOFrequency;
  friend class TinTinModulationFrequencyParameter;
  friend class TintinUnambiguousRangeParameter;
};

}
}

#endif // VOXEL_TI_TOFTINTINCAMERA_H
