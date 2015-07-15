/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TINTINCDKCAMERA_H
#define VOXEL_TI_TINTINCDKCAMERA_H

#include <ToFTintinCamera.h>
#include <Downloader.h>

#define TINTIN_CDK_VENDOR_ID 0x0451U
#define TINTIN_CDK_PRODUCT_BULK 0x9105U
#define TINTIN_CDK_PRODUCT_UVC 0x9106U

#define PVDD "pvdd" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage
#define TILLUM_SLAVE_ADDR "tillum_slv_addr"
#define LUMPED_DEAD_TIME "lumped_dead_time"
#define ILLUM_DC_CORR "illum_dc_corr"
#define ILLUM_DC_CORR_DIR "illum_dc_corr_dir"
#define DELAY_FB_CORR_MODE "delay_fb_corr_mode"
#define DELAY_FB_DC_CORR_MODE "delay_fb_dc_corr_mode"
#define COMP_VREF "comp_vref"

#define TINTIN_CDK_USBIO_BOARD_REVISION 0x30

namespace Voxel
{
  
namespace TI
{

class TintinCDKCamera: public ToFTintinCamera
{
protected:
  Ptr<Downloader> _downloader;
  
  uint8_t _boardRevision[2];
  
  bool _init();
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  virtual bool _setStreamerFrameSize(const FrameSize &s);
  
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const;
  virtual bool _getMaximumFrameRate(FrameRate& frameRate, const FrameSize& forFrameSize) const;
  
  virtual bool _initStartParams();
  
  virtual bool _readConfigFromHardware(MainConfigurationFile::ConfigSerialNumberType &serialNumber, TimeStampType& knownTimestamp, Voxel::SerializedObject& so);
  virtual bool _writeConfigFromHardware(MainConfigurationFile::ConfigSerialNumberType &serialNumber, TimeStampType& timestamp, SerializedObject &so);
  
  virtual bool _getEEPROMSize(uint32_t &size);
  
public:
  TintinCDKCamera(DevicePtr device);
  
  virtual ~TintinCDKCamera() {}
  
};

}
}

#endif // VOXEL_TI_TINTINCDKCAMERA_H
