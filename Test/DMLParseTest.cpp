/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include <iomanip>
#include <fstream>

#include <TinyXML2.h>
#include <ParameterDMLParser.h>
#include <VoxelXUProgrammer.h>

using namespace Voxel;
using namespace TinyXML2;

enum Options
{
  XML_FILE = 0,
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { XML_FILE,    "-x", SO_REQ_SEP, "XML file name"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "DMLParseTest v1.0" << std::endl;
  
  CSimpleOpt::SOption *option = argumentSpecifications.data();
  
  while(option->nId >= 0)
  {
    std::cout << option->pszArg << " " << option->helpInfo << std::endl;
    option++;
  }
}


int main(int argc, char *argv[])
{
  CSimpleOpt s(argc, argv, argumentSpecifications);
  
  logger.setDefaultLogLevel(DEBUG);
  
  String xmlFile;
  
  char *endptr;
  
  while (s.Next())
  {
    if (s.LastError() != SO_SUCCESS)
    {
      std::cout << s.GetLastErrorText(s.LastError()) << ": '" << s.OptionText() << "' (use -h to get command line help)" << std::endl;
      help();
      return -1;
    }
    
    //std::cout << s.OptionId() << ": " << s.OptionArg() << std::endl;
    
    Vector<String> splits;
    switch (s.OptionId())
    {
      case XML_FILE:
        xmlFile = s.OptionArg();
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(xmlFile.size() == 0)
  {
    logger(LOG_ERROR) << "Required argument missing." << endl;
    help();
    return -1;
  }
  
  std::ifstream f(xmlFile, std::ios::binary);
  
  if(!f.good())
  {
    logger(LOG_ERROR) << "Failed to open '" << xmlFile << "'" << endl;
    return -1;
  }
  
  f.close();
  
  //XMLDocument doc;
  
  //doc.LoadFile(xmlFile.c_str());
  
  //XMLElement *node = doc.FirstChildElement("RegMap");
  
  DevicePtr d(new USBDevice(0x0, 0x0, "")); // dummy device
  
  TI::VoxelXUProgrammer r(d); // dummy register programmer
  
  ParameterDMLParser p(r, xmlFile);
  
  Vector<ParameterPtr> params;
  
  p.getParameters(params);
  
  return 0;
  
}