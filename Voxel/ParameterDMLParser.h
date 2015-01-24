/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PARAMETERDMLPARSER_H
#define VOXEL_PARAMETERDMLPARSER_H

#include "TinyXML2.h"
#include "Common.h"
#include "Parameter.h"

namespace Voxel
{
/**
 * \addtogroup Param
 * @{
 */


class VOXEL_EXPORT ParameterDMLParser
{
protected:
  TinyXML2::XMLDocument _doc;
  
  TinyXML2::XMLElement *_regMap, *_propertyList, *_sectionList;
  
  uint8_t _wordLength;
  
  String _xmlFileName;
  
  RegisterProgrammer &_registerProgrammer;
  
  bool _initialized = false;
  
  bool _prepare();
  
  TinyXML2::XMLElement *_goTo(TinyXML2::XMLElement *current, const Vector<String> &nodeNameList, bool report = true); // by default report error
  
  ParameterPtr _getParameter(TinyXML2::XMLElement *property, String &id, bool &skipIfNull); // Get parameter and ID
  
public:
  ParameterDMLParser(RegisterProgrammer &registerProgrammer, const String &xmlFileName);
  
  inline bool isInitialized() { return _initialized; }
  
  bool getParameters(Vector<ParameterPtr> &parameters);
  
  virtual ~ParameterDMLParser() {}
};

/**
 * @}
 */

}

#endif // PARAMETERDMLPARSER_H
