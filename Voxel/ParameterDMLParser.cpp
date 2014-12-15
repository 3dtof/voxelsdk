/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ParameterDMLParser.h"


namespace Voxel
{
  
ParameterDMLParser::ParameterDMLParser(RegisterProgrammer &registerProgrammer, const String &xmlFileName): _initialized(false), _xmlFileName(xmlFileName), _registerProgrammer(registerProgrammer)
{
  _doc.LoadFile(xmlFileName.c_str());
  
  if(_doc.Error())
  {
    logger(ERROR) << "ParameterDMLParser: Error parsing the XML file '" << xmlFileName.c_str() << "'" << std::endl;
    _initialized = false;
    return;
  }
  
  _initialized = true;
}

bool ParameterDMLParser::_prepare()
{
  _regMap = _doc.FirstChildElement();
  
  if(_regMap->Attribute("name") and _regMap->Attribute("version"))
  {
    logger(INFO) << "ParameterDMLParser: Found register map with name '" << _regMap->Attribute("name") << "' (v" << _regMap->Attribute("version") << ")" << std::endl;
  }
  
  
  if(!(_propertyList = _goTo(_regMap, { "propertyViewList", "propertyView", "propertyList" })))
  {
    logger(ERROR) << "ParameterDMLParser: Could not get 'propertyList' from the DML file '" << _xmlFileName << "'" << std::endl;
    return false;
  }
    
  if(!(_sectionList = _goTo(_regMap, { "guiViewList", "guiView", "sectionList" }))) // Picks the first guiView
  {
    logger(ERROR) << "ParameterDMLParser: Could not get 'sectionList' from the DML file '" << _xmlFileName << "'" << std::endl;
    return false;
  }
  
  _wordLength = _regMap->IntAttribute("wordLength");
  
  if(!_wordLength)
  {
    logger(ERROR) << "ParameterDMLParser: Got wordLength to be zero. Please check DML file '" << _xmlFileName << "'" << std::endl;
  }
  
  return true;
}

TinyXML2::XMLElement *ParameterDMLParser::_goTo(TinyXML2::XMLElement *current, const Vector<String> &nodeNameList, bool report)
{
  TinyXML2::XMLElement *x = current;
  
  for(auto &g: nodeNameList)
  {
    x = x->FirstChildElement(g.c_str());
    
    if(!x)
    {
      if(report)
        logger(ERROR) << "ParameterDMLParser: In XML file '" << _xmlFileName.c_str() << "'. Could not get '" << g << "'" << std::endl;
      return 0;
    }
  }
  
  return x;
}

ParameterPtr ParameterDMLParser::_getParameter(TinyXML2::XMLElement *property, String &id, bool &skipIfNull)
{
  TinyXML2::XMLElement *range, *value;
  
  skipIfNull = false;
  
  const char *s;
  
  if(!(s = property->Attribute("id")))
  {
    logger(ERROR) << "Found a 'property' with no 'id'. " << property->GetText() << std::endl;
    return 0;
  }
  
  id = s;
  
  if(id == "693")
    std::cout << std::endl;
  
  uint bitCount = property->UnsignedAttribute("totalBits"),
    min = property->UnsignedAttribute("min"),
    max = property->UnsignedAttribute("max");
  
  if(!(range = _goTo(property, { "rangeList", "range" }, false)))
  {
    if(bitCount != 0)
      logger(ERROR) << "ParameterDMLParser: Could not get 'range' for parameter with id = '" << id << "' with bitCount = " << bitCount << std::endl;
    skipIfNull = true;
    return 0; // No range => no valid parameter
  }
  
  uint address = range->UnsignedAttribute("address"),
    msb = range->UnsignedAttribute("msb"),
    lsb = range->UnsignedAttribute("lsb"),
    bankId = range->UnsignedAttribute("bankId");

  if(address >= 256)
  {
    logger(ERROR) << "ParameterDMLParser: Address value = '" << address << "' which is beyond 256 for parameter with id = '" << id << "'" << std::endl;
    return 0;
  }
  
  address = (bankId << 8) + address;
        
  if(msb < lsb)
  {
    logger(ERROR) << "ParameterDMLParser: Found a parameter with id ='" << id << "' which has msb (" << msb << ") < lsb(" << lsb << ")" << std::endl;
    return 0;
  }
  
  bool strobe = property->BoolAttribute("strobe"),
        sign = property->BoolAttribute("signed"),
        readOnly = property->BoolAttribute("readonly");
        
        
  Parameter::IOType ioType = readOnly?Parameter::IO_READ_ONLY:Parameter::IO_READ_WRITE;
  
  String units, description;
  
  if((s = property->Attribute("units")))
    units = s;
  
  if((s = property->Attribute("desc")))
    units = s;
  
  if(!(value = _goTo(property, { "valueList", "value" }, false))) // No value list present? => not enum
  {
    if(bitCount == 1) // This boolean does not have value descriptions :(
    {
      logger(DEBUG) << "ParameterDMLParser: Found a boolean parameter with id = '" << id << "' which does not have valueList" << std::endl;
      
      bool defaultValue = property->BoolAttribute("default");
      
      if(!strobe)
      {
        logger(DEBUG) << "ParameterDMLParser: Adding boolean parameter with id = '" << id << "'" << std::endl;
        return ParameterPtr(new BoolParameter(_registerProgrammer, "", address, _wordLength, lsb, {"", ""}, {"", ""}, defaultValue, "", description,
                                            ioType));
      }
      else
      {
        logger(DEBUG) << "ParameterDMLParser: Adding strobe (boolean) parameter with id = '" << id << "'" << std::endl;
        return ParameterPtr(new StrobeBoolParameter(_registerProgrammer, "", address, _wordLength, lsb, {"", ""}, {"", ""}, defaultValue, "", description,
                                            ioType));
      }
    }
    else if(bitCount > 1)
    {
      if(strobe)
        logger(WARNING) << "ParameterDMLParser: Found a non-boolean parameter with id ='" << id << "' marked as strobe. Ignoring 'strobe' for this." << std::endl;
      
      if(sign)
      {
        if((int)max < (int)min)
        {
          logger(ERROR) << "ParameterDMLParser: Found a parameter with id ='" << id << "' which has max (" << max << ") < min(" << min << ")" << std::endl;
          return 0;
        }
        
        logger(DEBUG) << "ParameterDMLParser: Adding int parameter with id = '" << id << "'" << std::endl;
        int defaultValue = property->IntAttribute("default");
        return ParameterPtr(new IntegerParameter(_registerProgrammer, "", units, address, _wordLength, msb, lsb, min, max, defaultValue, "", description,
                                              ioType));
      }
      else
      {
        if(max < min)
        {
          logger(ERROR) << "ParameterDMLParser: Found a parameter with id ='" << id << "' which has max (" << max << ") < min(" << min << ")" << std::endl;
          return 0;
        }
        
        logger(DEBUG) << "ParameterDMLParser: Adding unsigned int parameter with id = '" << id << "'" << std::endl;
        uint defaultValue = property->UnsignedAttribute("default");
        return ParameterPtr(new UnsignedIntegerParameter(_registerProgrammer, "", units, address, _wordLength, msb, lsb, min, max, defaultValue, "", description,
                                                 ioType));
      }
    }
    else //bitCount == 0
    {
      logger(WARNING) << "ParameterDMLParser: Found a parameter with id ='" << id << "' which has bitCount = 0." << std::endl;
      skipIfNull = true;
      return 0;
    }
  }
  else // value list is present => enum
  {
    if(bitCount == 1) // boolean
    {
      bool defaultValue = property->BoolAttribute("default");
      
      Vector<String> valueMeaning, valueDescription;
      
      valueMeaning.resize(2);
      valueDescription.resize(2);
      
      for(; value; value = value->NextSiblingElement())
      {
        String m, d;
        
        if((s = value->Attribute("meaning")))
          m = s;
        
        if((s = value->Attribute("desc")))
          d = s;
        
        int v = value->IntAttribute("value");
        
        if(v == 0 || v == 1)
        {
          valueMeaning[v] = m;
          valueDescription[v] = d;
        }
        else
        {
          logger(ERROR) << "ParameterDMLParser: Found a boolean parameter with id = '" << id << "' which has possible value as '" << v << "'" << std::endl;
          return 0;
        }
      }
      
      if(!strobe)
      {
        logger(DEBUG) << "ParameterDMLParser: Adding boolean parameter with id = '" << id << "'" << std::endl;
        return ParameterPtr(new BoolParameter(_registerProgrammer, "", address, _wordLength, lsb, valueDescription, valueMeaning, defaultValue, "", description,
                                              ioType));
      }
      else
      {
        logger(DEBUG) << "ParameterDMLParser: Adding strobe (boolean) parameter with id = '" << id << "'" << std::endl;
        return ParameterPtr(new StrobeBoolParameter(_registerProgrammer, "", address, _wordLength, lsb, valueDescription, valueMeaning, defaultValue, "", description,
                                              ioType));
      }
    }
    else if(bitCount > 1)
    {
      if(strobe)
        logger(WARNING) << "ParameterDMLParser: Found a non-boolean parameter with id ='" << id << "' marked as strobe. Ignoring 'strobe' for this." << std::endl;
      
      Vector<String> valueMeaning, valueDescription;
      Vector<int> values;
      
      for(; value; value = value->NextSiblingElement())
      {
        String m, d;
        
        if((s = value->Attribute("meaning")))
          m = s;
        
        if((s = value->Attribute("desc")))
          d = s;
        
        valueMeaning.push_back(m);
        valueDescription.push_back(d);
        values.push_back(value->IntAttribute("value"));
      }
      
      logger(DEBUG) << "ParameterDMLParser: Adding enum parameter with id = '" << id << "'" << std::endl;
      
      int defaultValue = property->IntAttribute("default");
      return ParameterPtr(new EnumParameter(_registerProgrammer, "", address, _wordLength, msb, lsb, values, valueDescription, valueMeaning, defaultValue, "", description,
                                            ioType));
    }
    else //bitCount == 0
    {
      logger(ERROR) << "ParameterDMLParser: Found a parameter with id ='" << id << "' which has bitCount = 0 but has value list." << std::endl;
      return 0;
    }
  }
}



bool ParameterDMLParser::getParameters(Vector<ParameterPtr> &parameters)
{
  if(!isInitialized())
    return false;
  
  if(!_prepare())
    return false;
  
  Map<String, ParameterPtr> paramMap; // id -> paramter
  
  String id;
  
  TinyXML2::XMLElement *x = _propertyList->FirstChildElement("property");
  
  ParameterPtr p;
  
  bool skipIfNull = false;
  
  for(; x; x = x->NextSiblingElement())
  {
    if(!(p = _getParameter(x, id, skipIfNull)) && !skipIfNull)
      return false;
    
    if(!p)
      continue;
    
    if(paramMap.find(id) != paramMap.end())
    {
      logger(ERROR) << "ParameterDMLParser: Found an existing parameter with id = '" << id << "'" << std::endl;
      return false;
    }
    
    paramMap[id] = p;
  }
  
  logger(DEBUG) << "ParameterDMLParser: Total number of valid parameters = " << paramMap.size() << std::endl;
  
  TinyXML2::XMLElement *section = _sectionList->FirstChildElement("section");
  
  char *s;
  String name;
  
  for(; section; section = section->NextSiblingElement())
  {
    if(!section->Attribute("name"))
    {
      logger(ERROR) << "ParameterDMLParser: Found a section with no name in DML file '" << _xmlFileName << std::endl;
      return false;
    }
    
    TinyXML2::XMLElement *group = _goTo(section, { "groupList", "group" }, false);
    
    for(; group; group = group->NextSiblingElement())
    {
      if(!group->Attribute("name"))
      {
        logger(ERROR) << "ParameterDMLParser: Found a group with no name in section " << section->Attribute("name") << " in DML file '" << _xmlFileName << std::endl;
        return false;
      }
      
      TinyXML2::XMLElement *property = _goTo(group, { "propertyList", "property" }, false);
      
      for(; property; property = property->NextSiblingElement())
      {
        if(!property->Attribute("name") || !property->Attribute("propertyId"))
        {
          logger(ERROR) << "ParameterDMLParser: Found a property with no name or propertyId, in " << section->Attribute("name") << "." << group->Attribute("name") << std::endl;
          return false;
        }
        
        name = property->Attribute("name");
        id = property->Attribute("propertyId");
        
        auto p = paramMap.find(id);
        
        if(p != paramMap.end())
        {
          p->second->setName(name);
          parameters.push_back(p->second);
        }
        else if(name != "...")
        {
          logger(ERROR) << "ParameterDMLParser: Could not find a parameter with id = " << id << ", in " << section->Attribute("name") << "." << group->Attribute("name") << std::endl;
          return false;
        }
      }
    }
  }
  
  logger(DEBUG) << "ParameterDMLParser: Total number of valid parameters = " << parameters.size() << std::endl;
  
  return true;
}

  
}