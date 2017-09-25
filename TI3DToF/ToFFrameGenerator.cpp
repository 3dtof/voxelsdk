/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <ToFFrameGenerator.h>

#include <ToFCamera.h>

#define _MATH_DEFINES
#include <math.h>

#define MAX_PHASE_VALUE 0x0FFF
#define MAX_PHASE_RANGE 0x1000
#define TOF_ANGLE_TO_PHASE_FACTOR 4096/(2*M_PI)
#define IQ_SIGN_BIT 0x0800
#define IQ_SIGN_EXTEND 0xF000

#define PARAM_TOF_FRAME_TYPE "tofFrameType"
#define PARAM_PHASE_OFFSETS_FILE "phaseOffsetsFile"
#define PARAM_PHASE_OFFSETS "phaseOffsets"
#define PARAM_CROSS_TALK_COEFF "crossTalkCoefficients"
#define PARAM_BYTES_PER_PIXEL "bytesPerPixel"
#define PARAM_DATA_ARRANGE_MODE "dataArrangeMode"
#define PARAM_ROI_X "roiX"
#define PARAM_ROI_Y "roiY"
#define PARAM_ROI_WIDTH "roiWidth"
#define PARAM_ROI_HEIGHT "roiHeight"
#define PARAM_MAX_FRAME_WIDTH "maxFrameWidth"
#define PARAM_MAX_FRAME_HEIGHT "maxFrameHeight"
#define PARAM_ROWS_TO_MERGE "rowsToMerge"
#define PARAM_COLUMNS_TO_MERGE "columnsToMerge"
#define PARAM_HISTOGRAM_ENABLED "histogramEnabled"

#define PARAM_QUAD_COUNT "quadCount"

#define PARAM_FRAME_WIDTH "frameWidth"
#define PARAM_FRAME_HEIGHT "frameHeight"

#define PARAM_DEALIASED_16BIT_MODE "dealiased16BitMode"

/* Parameter for enabling/disabling phase offsets.*/
#define PARAM_PHASE_OFFSETS_DISABLE "disablePhaseOffsets"

/* Parameters for enabling dealiasing in calculus */
#define PARAM_DATA_TO_REPLACE "dataToReplace"

namespace Voxel
{
  
namespace TI
{
  
ToFFrameGenerator::ToFFrameGenerator(): 
  FrameGenerator((TI_VENDOR_ID << 16) | DepthCamera::FRAME_RAW_FRAME_PROCESSED, DepthCamera::FRAME_RAW_FRAME_PROCESSED, 0, 6),
_bytesPerPixel(-1), _dataArrangeMode(-1), _histogramEnabled(false)
{
  _frameGeneratorParameters[PARAM_BYTES_PER_PIXEL] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_DATA_ARRANGE_MODE] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_TOF_FRAME_TYPE] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROI_X] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROI_Y] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROI_WIDTH] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROI_HEIGHT] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_MAX_FRAME_WIDTH] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_MAX_FRAME_HEIGHT] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_FRAME_WIDTH] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_FRAME_HEIGHT] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_ROWS_TO_MERGE] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_COLUMNS_TO_MERGE] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_HISTOGRAM_ENABLED] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_QUAD_COUNT] = SerializablePtr(new SerializableUnsignedInt());
  
  _frameGeneratorParameters[PARAM_PHASE_OFFSETS_FILE] = SerializablePtr(new SerializableString());
  _frameGeneratorParameters[PARAM_PHASE_OFFSETS] = SerializablePtr(new SerializableString());
  _frameGeneratorParameters[PARAM_CROSS_TALK_COEFF] = SerializablePtr(new SerializableString());
  
  _frameGeneratorParameters[PARAM_DEALIASED_16BIT_MODE] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_PHASE_OFFSETS_DISABLE] = SerializablePtr(new SerializableUnsignedInt());
  _frameGeneratorParameters[PARAM_DATA_TO_REPLACE] = SerializablePtr(new SerializableUnsignedInt());
}

bool ToFFrameGenerator::_onWriteConfiguration()
{
  if(_bytesPerPixel == -1)
    return false;
  return true;
}

bool ToFFrameGenerator::_onReadConfiguration()
{
  uint32_t frameType;
  if(!get(PARAM_TOF_FRAME_TYPE, frameType) ||
    !get(PARAM_BYTES_PER_PIXEL, _bytesPerPixel) ||
    !get(PARAM_DATA_ARRANGE_MODE, _dataArrangeMode) ||
    !get(PARAM_ROI_X, _roi.x) ||
    !get(PARAM_ROI_Y, _roi.y) ||
    !get(PARAM_ROI_WIDTH, _roi.width) ||
    !get(PARAM_ROI_HEIGHT, _roi.height) ||
    !get(PARAM_CROSS_TALK_COEFF, _crossTalkCoefficients) ||
    !get(PARAM_HISTOGRAM_ENABLED, _histogramEnabled) ||
    !get(PARAM_MAX_FRAME_WIDTH, _maxFrameSize.width) ||
    !get(PARAM_MAX_FRAME_HEIGHT, _maxFrameSize.height) ||
    !get(PARAM_ROWS_TO_MERGE, _rowsToMerge) ||
    !get(PARAM_COLUMNS_TO_MERGE, _columnsToMerge))
    return false;
    
  if(_majorVersion == 0 && _minorVersion < 2)
  {
    _dealiased16BitMode = false;
  }
  else
  {
    uint32_t d;
    if(!get(PARAM_DEALIASED_16BIT_MODE, d))
      return false;
    _dealiased16BitMode = d > 0;
  }
  
  if(_majorVersion == 0 && _minorVersion < 3)
  {
    _size.width = (_roi.width + _columnsToMerge - 1)/_columnsToMerge;
    _size.height = (_roi.height + _rowsToMerge - 1)/_rowsToMerge;
    _frameSize = _size;
    _quadCount = 4;
  }
  else
  {
    if(!get(PARAM_FRAME_WIDTH, _frameSize.width) || !get(PARAM_FRAME_HEIGHT, _frameSize.height) ||
      !get(PARAM_QUAD_COUNT, _quadCount))
      return false;
    
    _size.width = std::min<uint>(_frameSize.width, (_roi.width + _columnsToMerge - 1)/_columnsToMerge);
    _size.height = std::min<uint>(_frameSize.height, (_roi.height + _rowsToMerge - 1)/_rowsToMerge);
  }
  
  _frameType = (ToFFrameType)frameType;
  
  if(_majorVersion == 0 && _minorVersion < 4)
  {
    if(!get(PARAM_PHASE_OFFSETS_FILE, _phaseOffsetFileName) || !_readPhaseOffsetCorrection())
      return false;
  }
  else
  {
    String p;
    if(!get(PARAM_PHASE_OFFSETS, p))
      return false;
    
    _phaseOffsetCorrectionData.resize((p.size() + 1)/2);
    memcpy(_phaseOffsetCorrectionData.data(), p.data(), p.size());
  }

  if(_majorVersion == 0 && _minorVersion < 6)
  {
    _dataToReplace = AMBIENT_DATA;
  }
  else
  {
    if(!get(PARAM_DATA_TO_REPLACE, _dataToReplace))
      return false;
  }
  
	_disablePhaseOffsets = false;

  if(!_createCrossTalkFilter())
    return false;
  return true;
}

bool ToFFrameGenerator::setParameters(const String &phaseOffsetFileName, const Vector<int16_t> &phaseOffsets, uint32_t bytesPerPixel, 
                                      uint32_t dataArrangeMode, 
                                      const RegionOfInterest &roi, const FrameSize &maxFrameSize, 
                                      const FrameSize &frameSize,
                                      uint rowsToMerge, uint columnsToMerge,
                                      uint8_t histogramEnabled, const String &crossTalkCoefficients,
                                      ToFFrameType type, 
                                      uint32_t quadCount,
                                      bool dealiased16BitMode,
                                      int dealiasedPhaseMask,
                                      bool disablePhaseOffsets, uint32_t dataToReplace)
{
  if(_phaseOffsetFileName == phaseOffsetFileName && bytesPerPixel == _bytesPerPixel && 
    _dataArrangeMode == dataArrangeMode && 
    _maxFrameSize == maxFrameSize && _frameSize == frameSize &&
    _roi == roi && _rowsToMerge == rowsToMerge && _columnsToMerge == columnsToMerge
    && _histogramEnabled == histogramEnabled &&
    _crossTalkCoefficients == crossTalkCoefficients && 
    _frameType == type && _quadCount == quadCount &&
  _dealiased16BitMode == dealiased16BitMode && _dealiasedPhaseMask == dealiasedPhaseMask &&
  _disablePhaseOffsets == disablePhaseOffsets && _dataToReplace == dataToReplace)

    return true;
  
  _phaseOffsetFileName = phaseOffsetFileName;
  _phaseOffsetCorrectionData = phaseOffsets;
  _dealiasedPhaseMask = dealiasedPhaseMask;
  _disablePhaseOffsets = disablePhaseOffsets;
  _dataToReplace = dataToReplace;
  
  if(phaseOffsets.size() == maxFrameSize.width*maxFrameSize.height + 2) // Ignore the first two elements if present
  {
    Vector<int16_t>(_phaseOffsetCorrectionData.begin() + 2, _phaseOffsetCorrectionData.end()).swap(_phaseOffsetCorrectionData);
    _dealiasedPhaseMaskInPhaseOffset = 0;
  }
  else if(phaseOffsets.size() == maxFrameSize.width*maxFrameSize.height + 3)
  {
    // Lower 4 bits
    _dealiasedPhaseMaskInPhaseOffset = _phaseOffsetCorrectionData[2] & 0x000F;
    
    if(_dealiasedPhaseMaskInPhaseOffset > 7)
      _dealiasedPhaseMaskInPhaseOffset = _dealiasedPhaseMaskInPhaseOffset - 16;
    
    Vector<int16_t>(_phaseOffsetCorrectionData.begin() + 3, _phaseOffsetCorrectionData.end()).swap(_phaseOffsetCorrectionData);
  }
  else
    _dealiasedPhaseMaskInPhaseOffset = _dealiasedPhaseMask;
  
  std::cout << "_dealiasedPhaseMask = " << _dealiasedPhaseMask 
    << ", _dealiasedPhaseMaskInPhaseOffset = " << _dealiasedPhaseMaskInPhaseOffset << std::endl;
  
  if(_dealiasedPhaseMask > _dealiasedPhaseMaskInPhaseOffset)
  {
    for(auto i = 0; i < _phaseOffsetCorrectionData.size(); i++)
      _phaseOffsetCorrectionData[i] <<= (_dealiasedPhaseMask - _dealiasedPhaseMaskInPhaseOffset);
  }
  else
  {
    for(auto i = 0; i < _phaseOffsetCorrectionData.size(); i++)
      _phaseOffsetCorrectionData[i] >>= (_dealiasedPhaseMaskInPhaseOffset - _dealiasedPhaseMask);
  }
  
  
  String phaseOffsetsString;
  
  if(phaseOffsets.size() > 0)
  {
    phaseOffsetsString.resize(_phaseOffsetCorrectionData.size()*2);
    memcpy(&phaseOffsetsString[0], _phaseOffsetCorrectionData.data(), phaseOffsetsString.size());
  }
  
  _bytesPerPixel = bytesPerPixel;
  _dataArrangeMode = dataArrangeMode;
  _maxFrameSize = maxFrameSize;
  _roi = roi;
  _rowsToMerge = rowsToMerge;
  _columnsToMerge = columnsToMerge;
  
  if(_roi.x < 0 || _roi.y < 0 || _roi.width < 0 || _roi.height < 0 || _maxFrameSize.width < 0
    || _maxFrameSize.height < 0 ||
    _roi.x + _roi.width > _maxFrameSize.width || _roi.y + _roi.height > _maxFrameSize.height ||
    _rowsToMerge < 1 || _columnsToMerge < 1)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Incorrect ROI or maxFrameSize or rowsToMerge or columnsToMerge" 
      << "ROI = (" << _roi.x << ", " << _roi.y << ", " << _roi.width << ", " << _roi.height << "), "
      << "maxFrameSize = (" << _maxFrameSize.width << ", " << _maxFrameSize.height << "), "
      << "rowsToMerge = " << _rowsToMerge << ", columnsToMerge = " << _columnsToMerge << std::endl;
    return false;
  }
  
  _frameSize = frameSize;
  _size.width = std::min<uint>(frameSize.width, (_roi.width + _columnsToMerge - 1)/_columnsToMerge);
  _size.height = std::min<uint>(frameSize.height, (_roi.height + _rowsToMerge - 1)/_rowsToMerge);
  
  _histogramEnabled = histogramEnabled;
  _crossTalkCoefficients = crossTalkCoefficients;
  _frameType = type;
  _dealiased16BitMode = dealiased16BitMode;
  
  _quadCount = quadCount;
  
  if(_quadCount > 100) // Something not right about quad count
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Incorrect quad count = " << _quadCount << std::endl;
    return false;
  }
  
  if(_frameType == ToF_QUAD)
  {
    _sineTable.resize(_quadCount);
    _cosineTable.resize(_quadCount);
    
    for(auto i = 0; i < _quadCount; i++)
    {
      _sineTable[i] = sin(2*M_PI/_quadCount*i);
      _cosineTable[i] = cos(2*M_PI/_quadCount*i);
      logger(LOG_INFO) << "table @" << i << " = " << _sineTable[i] << ", " << _cosineTable[i] << std::endl;
    }
  }
  
  if(!_createCrossTalkFilter())
    return false;
  
  uint32_t frameType = type;
  uint32_t d = _dealiased16BitMode;
  uint32_t p = _disablePhaseOffsets;

  if(
    !_set(PARAM_TOF_FRAME_TYPE, frameType) ||
    !_set(PARAM_QUAD_COUNT, quadCount) ||
    !_set(PARAM_BYTES_PER_PIXEL, _bytesPerPixel) ||
    !_set(PARAM_DATA_ARRANGE_MODE, _dataArrangeMode) ||
    !_set(PARAM_ROI_X, _roi.x) ||
    !_set(PARAM_ROI_Y, _roi.y) ||
    !_set(PARAM_ROI_WIDTH, _roi.width) ||
    !_set(PARAM_ROI_HEIGHT, _roi.height) ||
    !_set(PARAM_PHASE_OFFSETS_FILE, _phaseOffsetFileName) ||
    !_set(PARAM_PHASE_OFFSETS, phaseOffsetsString) ||
    !_set(PARAM_CROSS_TALK_COEFF, _crossTalkCoefficients) ||
    !_set(PARAM_HISTOGRAM_ENABLED, _histogramEnabled) ||
    !_set(PARAM_MAX_FRAME_WIDTH, _maxFrameSize.width) ||
    !_set(PARAM_MAX_FRAME_HEIGHT, _maxFrameSize.height) ||
    !_set(PARAM_FRAME_WIDTH, _frameSize.width) ||
    !_set(PARAM_FRAME_HEIGHT, _frameSize.height) ||
    !_set(PARAM_ROWS_TO_MERGE, _rowsToMerge) ||
    !_set(PARAM_COLUMNS_TO_MERGE, _columnsToMerge) ||
    !_set(PARAM_DEALIASED_16BIT_MODE, d) ||
    !_set(PARAM_PHASE_OFFSETS_DISABLE, p) ||
    !_set(PARAM_DATA_TO_REPLACE, _dataToReplace))
    return false;
  
  return writeConfiguration();
}

bool ToFFrameGenerator::_createCrossTalkFilter()
{
  if(_crossTalkCoefficients.size())
  {
    _crossTalkFilter = ToFCrossTalkFilterPtr(new ToFCrossTalkFilter());
    
    if(!_crossTalkFilter->readCoefficients(_crossTalkCoefficients))
    {
      _crossTalkFilter = nullptr;
      
      logger(LOG_ERROR) << "ToFFrameGenerator: Failed to read cross talk filter coefficients. Coefficients = "
      << _crossTalkCoefficients << std::endl;
      
      return false;
    }
    
    if(!_crossTalkFilter->setMaxPhaseRange(MAX_PHASE_RANGE))
      return false;
  }
  else
    _crossTalkFilter = nullptr;
  
  return true;
}


bool ToFFrameGenerator::_readPhaseOffsetCorrection()
{
  if(!_phaseOffsetFileName.size())
  {
    _phaseOffsetCorrectionData.clear();
    return true;
  }
  
  Configuration c;
  
  String file = _phaseOffsetFileName;
  
  if(!c.getConfFile(file))
    return false;
  
  InputFileStream f(file, std::ios::binary | std::ios::ate);
  
  if(!f.good())
    return false;
  
  size_t size = f.tellg();
  
  _phaseOffsetCorrectionData.resize(size/2); // int16_t data
  
  f.seekg(0, std::ios::beg);
  f.clear();
  
  if(!f.good())
    return false;
  
  f.read((char *)_phaseOffsetCorrectionData.data(), size);
  return true;
}

bool ToFFrameGenerator::_applyPhaseOffsetCorrection(Vector<uint16_t> &phaseData)
{
  if(!_phaseOffsetCorrectionData.size() || _disablePhaseOffsets)
    return true; // Nothing to do
  
  int16_t *_phaseOffsetCorrection;
    
  if(_phaseOffsetCorrectionData.size() == _maxFrameSize.height*_maxFrameSize.width)
    _phaseOffsetCorrection = _phaseOffsetCorrectionData.data();
  else if(_phaseOffsetCorrectionData.size() == _maxFrameSize.height*_maxFrameSize.width + 2) // contains rows and columns information?
    _phaseOffsetCorrection = _phaseOffsetCorrectionData.data() + 2;
  else
    return false;
  
  if(phaseData.size() != (_size.width)*(_size.height))
    return false;
  
  int i = _size.height, j;
  
  int16_t v;
  uint16_t *d = phaseData.data();
  int16_t *o = _phaseOffsetCorrection + _roi.x + _roi.y*_maxFrameSize.width, *o1;
  o1 = o;
  
  
  while(i--)
  {
    j = _size.width;
    while(j--)
    {
      v = *d - *o;
      if(v < 0)
        *d = 0;
      else if(v >= MAX_PHASE_VALUE + 1)
        *d = MAX_PHASE_VALUE;
      else
        *d = v;
      d++;
      o += _columnsToMerge;
    }
    o1 += _rowsToMerge*_maxFrameSize.width;
    o = o1;
  }
  
  return true;
}

bool ToFFrameGenerator::generate(const FramePtr &in, FramePtr &out)
{
  if(_frameType == ToF_I_Q || _frameType == ToF_QUAD)
    return _generateToFRawIQFrame(in, out);
  else
    return _generateToFRawFrame(in, out);
}

bool ToFFrameGenerator::_generateToFRawFrame(const FramePtr &in, FramePtr &out)
{
  RawDataFramePtr rawDataFrame = std::dynamic_pointer_cast<RawDataFrame>(in);
  
  if(!rawDataFrame)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Input data frame is not of raw data type." << std::endl;
    return false;
  }
  
  ToFRawFrameTemplate<uint16_t, uint8_t> *t;
  if(!_crossTalkFilter)
  {
    t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(out.get());
    
    if(!t)
    {
      t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
      out = FramePtr(t);
    }
  }
  else
  {
    t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(_filterInputFrame.get());
    
    if(!t)
    {
      t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
      _filterInputFrame = FramePtr(t);
    }
  }
  
  t->size = _size;
  t->id = rawDataFrame->id;
  t->timestamp = rawDataFrame->timestamp;
  
  if(_bytesPerPixel == 4)
  {
    if(rawDataFrame->data.size() < _size.height*_size.width*4)
    {
      logger(LOG_ERROR) << "ToFFrameGenerator: Incomplete raw data size = " << rawDataFrame->data.size() << ". Required size = " << _size.height*_size.width*4 << std::endl;
      return false;
    }
    
    if(_dataArrangeMode != 2 && _dataArrangeMode != 0)
    {
      logger(LOG_ERROR) << "ToFFrameGenerator: Invalid op_data_arrange_mode = " << _dataArrangeMode << " for pixel_data_size = " << _bytesPerPixel << std::endl;
      return false;
    }
    
    unsigned int frameSize = _size.width*_size.height;
    t->_ambient.resize(frameSize);
    t->_amplitude.resize(frameSize);
    t->_phase.resize(frameSize);
    t->_flags.resize(frameSize);
    
    if(_dataArrangeMode == 2)
    {
      auto index1 = 0, index2 = 0;
      
      uint16_t *data = (uint16_t *)rawDataFrame->data.data();
      
      for (auto i = 0; i < _size.height; i++) 
      {
        for (auto j = 0; j < _size.width/8; j++) 
        {
          index1 = i*_size.width*2 + j*16;
          index2 = i*_size.width + j*8;
          
          //logger(INFO) << "i = " << i << ", j = " << j << ", index1 = " << index1 << ", index2 = " << index2 << std::endl;
          
          for (auto k = 0; k < 8; k++) 
          {
            t->_amplitude[index2 + k] = data[index1 + k] & MAX_PHASE_VALUE;
            
            if(_dealiased16BitMode)
            {
              if (_dataToReplace == AMBIENT_DATA)
              {
                t->_phase[index2 + k] = ((data[index1 + k + 8] & MAX_PHASE_VALUE) << 4) + (data[index1 + k] >> 12);
                t->_ambient[index2 + k] = 0xF;
                t->_flags[index2 + k] = (data[index1 + k + 8] & 0xF000) >> 12;
              }

              else if (_dataToReplace == FLAGS_DATA)
              {
                t->_phase[index2 + k] = (data[index1 + k + 8]);
                t->_flags[index2 + k] = 0xF;
                t->_ambient[index2 + k] = (data[index1 + k] & 0xF000) >> 12;
              }
            }
            else
            {
              t->_phase[index2 + k] = data[index1 + k + 8] & MAX_PHASE_VALUE;
              t->_ambient[index2 + k] = (data[index1 + k] & 0xF000) >> 12;
              t->_flags[index2 + k] = (data[index1 + k + 8] & 0xF000) >> 12;
            }
          }
        }
      }
    }
    else // dataArrangeMode == 0
    {
      auto index1 = 0, index2 = 0;

      uint16_t *data = (uint16_t *)rawDataFrame->data.data();

      for (auto i = 0; i < _size.height; i++) 
      {
        for (auto j = 0; j < _size.width; j++) 
        {
          index1 = i*_size.width*2 + j*2;
          index2 = i*_size.width + j;
          t->_amplitude[index2] = data[index1] & MAX_PHASE_VALUE;

          if(_dealiased16BitMode)
          {
            if (_dataToReplace == AMBIENT_DATA)
            {
              t->_phase[index2] = ((data[index1 + 1] & MAX_PHASE_VALUE) << 4) + (data[index1] >> 12);
              t->_ambient[index2] = 0xF;
              t->_flags[index2] = (data[index1 + 1] & 0xF000) >> 12;
            }

            else if (_dataToReplace == FLAGS_DATA)
            {
              t->_phase[index2] = data[index1 + 1];
              t->_ambient[index2] = (data[index1] & 0xF000) >> 12;
              t->_flags[index2] = 0xF;
            }
          }
          else
          {
            t->_phase[index2] = data[index1 + 1] & MAX_PHASE_VALUE;
            t->_ambient[index2] = (data[index1] & 0xF000) >> 12;
            t->_flags[index2] = (data[index1 + 1] & 0xF000) >> 12;
          }
        }
      }
    }
  }
  else if(_bytesPerPixel == 2)
  {
    if(_dataArrangeMode != 0)
    {
      logger(LOG_ERROR) << "ToFFrameGenerator: " << OP_DATA_ARRANGE_MODE << " is expected to be zero, but got = " << _dataArrangeMode << " for " << PIXEL_DATA_SIZE << " = " << _bytesPerPixel << std::endl;
      return false;
    }
    
    unsigned int frameSize = _size.width*_size.height;
    if(rawDataFrame->data.size() < frameSize*2)
    {
      logger(LOG_ERROR) << "ToFFrameGenerator: Incomplete raw data size = " << rawDataFrame->data.size() << ". Required size = " << _size.height*_size.width*2 << std::endl;
      return false;
    }
    
    t->_ambient.resize(frameSize);
    t->_amplitude.resize(frameSize);
    t->_phase.resize(frameSize);
    t->_flags.resize(frameSize);

    auto index = 0;
    
    uint16_t *data = (uint16_t *)rawDataFrame->data.data();
    
    for (auto i = 0; i < _size.height; i++) 
    {
      for (auto j = 0; j < _size.width; j++) 
      {
        index = i*_size.width + j;
        t->_phase[index] = data[index] & MAX_PHASE_VALUE;
        t->_amplitude[index] = (data[index] & 0xF000) >> 4; // Amplitude information is MS 4-bits
        
        t->_ambient[index] = 0;
        t->_flags[index] = 0;
      }
    }
  }
  else
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Don't know to handle " << PIXEL_DATA_SIZE << " = " << _bytesPerPixel << std::endl;
    return false;
  }
  
  if(!_applyCrossTalkFilter(out))
    return false;


  t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(out.get());
  
  if(!t)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Output frame is not a ToF frame!" << std::endl;
    return false;
  }

  if(!_applyPhaseOffsetCorrection(t->_phase))
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Failed to apply phase offset correction" << std::endl;
    return false;
  }

  if(!_histogramEnabled)
    return true; // No histogram data
    
  if(rawDataFrame->data.size() < _size.height*_size.width*_bytesPerPixel + 96)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Histogram is enabled but raw data has less than 96 bytes at the end. Raw data size = " << rawDataFrame->data.size() 
    << ", bytes in frame = " << _size.height*_size.width*_bytesPerPixel << std::endl;
    return false;
  }

  uint8_t *data = rawDataFrame->data.data() + _size.height*_size.width*_bytesPerPixel;
  
  t->_histogram.resize(48); // 48 elements of 16-bits each

  memcpy((uint8_t *)t->_histogram.data(), data, 96);

  return true;
}

bool ToFFrameGenerator::generate(const ToFRawIQFramePtr &in, FramePtr &out)
{
  ToFRawIQFrameTemplate<int16_t> *input = dynamic_cast<ToFRawIQFrameTemplate<int16_t> *>(in.get());
  
  if(!input)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Input data frame is not of raw ToF IQ type." << std::endl;
    return false;
  }
  
  if(input->size != _size)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Input data size differs from expected size. Input size = " 
      << input->size.width << "x" << input->size.height 
      << ", expected size = " << _size.width << "x" << _size.height
      << std::endl;
    return false;
  }
  
  ToFRawFrameTemplate<uint16_t, uint8_t> *t;
  if(!_crossTalkFilter)
  {
    t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(out.get());
    
    if(!t)
    {
      t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
      out = FramePtr(t);
    }
  }
  else
  {
    t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(_filterInputFrame.get());
    
    if(!t)
    {
      t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
      _filterInputFrame = FramePtr(t);
    }
  }
  
  t->size = _size;
  t->id = input->id;
  t->timestamp = input->timestamp;
  
  unsigned int frameSize = _size.width*_size.height;
  t->_ambient.resize(frameSize);
  t->_amplitude.resize(frameSize);
  t->_phase.resize(frameSize);
  t->_flags.resize(frameSize);
  
  auto index = 0;
  
  Complex c;
  float phase;
  
  for (auto l = 0; l < _size.height; l++) 
  {
    for (auto j = 0; j < _size.width; j++) 
    {
      index = l*_size.width + j;
      
      c.real(input->_i[index]);
      c.imag(input->_q[index]);
      
      phase = std::arg(c);
      
      t->_phase[index] = ((phase < 0)?(2*M_PI + phase):phase)*TOF_ANGLE_TO_PHASE_FACTOR;
      t->_amplitude[index] = std::abs(c);
      
      t->_ambient[index] = 0;
      t->_flags[index] = 0;
    }
  }
  
  if(!_applyCrossTalkFilter(out))
    return false;
  
  
  t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(out.get());
  
  if(!t)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Output frame is not a ToF frame!" << std::endl;
    return false;
  }
  
  if(!_applyPhaseOffsetCorrection(t->_phase))
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Failed to apply phase offset correction" << std::endl;
    return false;
  }
  
  t->_histogram.clear(); // No histogram
  
  return true;
}


bool ToFFrameGenerator::_generateToFRawIQFrame(const FramePtr &in, FramePtr &out)
{
  if(_frameType == ToF_I_Q && _dataArrangeMode != 0)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Data arrange mode is expected to be zero. It is " << _dataArrangeMode << std::endl;
    return false;
  }
  
  RawDataFramePtr rawDataFrame = std::dynamic_pointer_cast<RawDataFrame>(in);
  
  if(!rawDataFrame)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Input data frame is not of raw data type." << std::endl;
    return false;
  }
  
  ToFRawIQFrameTemplate<int16_t> *t;
  
  t = dynamic_cast<ToFRawIQFrameTemplate<int16_t> *>(out.get());
    
  if(!t)
  {
    t = new ToFRawIQFrameTemplate<int16_t>();
    out = FramePtr(t);
  }
  
  t->size = _size;
  t->id = rawDataFrame->id;
  t->timestamp = rawDataFrame->timestamp;
  
  uint sizeFactor = 2;
  
  if(_frameType == ToF_QUAD)
    sizeFactor = _quadCount;
  
  unsigned int frameSize = _size.height*_size.width;
  if(rawDataFrame->data.size() < frameSize*sizeFactor)
  {
    logger(LOG_ERROR) << "ToFFrameGenerator: Incomplete raw data size = " << rawDataFrame->data.size() << ". Required size = " << _size.height*_size.width*2 << std::endl;
    return false;
  }
  
  t->_i.resize(frameSize);
  t->_q.resize(frameSize);
  
  auto index1 = 0, index2 = 0;
      
  uint16_t *data = (uint16_t *)rawDataFrame->data.data();
  
  if(_frameType == ToF_I_Q)
  {
    for (auto i = 0; i < _size.height; i++) 
    {
      for (auto j = 0; j < _size.width; j++) 
      {
        index1 = i*_size.width*2 + j*2;
        index2 = i*_size.width + j;
        
        t->_i[index2] = (int16_t)data[index1];
        t->_q[index2] = (int16_t)data[index1 + 1];
      }
    }
  }
  else if(_frameType == ToF_QUAD)
  {
    for (auto i = 0; i < _size.height; i++) 
    {
      for (auto j = 0; j < _size.width; j++) 
      {
        index2 = i*_size.width + j;
        index1 = index2*_quadCount;
        
        double ii = 0, iq = 0;
        int16_t d;
        
        for(auto k = 0; k < _quadCount; k++)
        {
          d = (((int16_t)(data[index1 + k] << 4)) >> 4); // Sign extension...
          ii += d*_cosineTable[k];
          iq += d*_sineTable[k];
        }
        
        //logger(LOG_INFO) << "@(" << j << ", " << i << ") = " << ii << ", " << iq << std::endl;
        
        t->_i[index2] = (int16_t)ii;
        t->_q[index2] = (int16_t)iq;
      }
    }
  }
  
  return true;
}

bool ToFFrameGenerator::_applyCrossTalkFilter(FramePtr &out)
{
  if(!_crossTalkFilter)
    return true;
  
  return _crossTalkFilter->filter(_filterInputFrame, out);
}


}
}
