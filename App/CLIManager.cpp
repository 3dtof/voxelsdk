/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "CLIManager.h"
#include <functional>

#include "LineNoise.h"
#include "PCLGrabber.h"

#define _C(F) std::bind(F, this, std::placeholders::_1, std::placeholders::_2)
#define _P(F) std::bind(F, this, std::placeholders::_1)
#define _H(F) std::bind(F, this)

#include <string.h>

#include <fstream>

using namespace LineNoise;

namespace Voxel
{
  
CLIManager::CLIManager(CameraSystem &sys): _sys(sys)
{
  _commands = Map<String, Command>({
    {"list",           Command(_H(&CLIManager::_listHelp),          _P(&CLIManager::_list),          nullptr)},
    {"status",         Command(_H(&CLIManager::_currentHelp),       _P(&CLIManager::_current),       nullptr)},
    {"connect",        Command(_H(&CLIManager::_connectHelp),       _P(&CLIManager::_connect),       _C(&CLIManager::_connectCompletion))},
    {"start",          Command(_H(&CLIManager::_startHelp),         _P(&CLIManager::_start),         nullptr)},
    {"stop",           Command(_H(&CLIManager::_stopHelp),          _P(&CLIManager::_stop),          nullptr)},
    {"getr",           Command(_H(&CLIManager::_getRegisterHelp),   _P(&CLIManager::_getRegister),   nullptr)},
    {"setr",           Command(_H(&CLIManager::_setRegisterHelp),   _P(&CLIManager::_setRegister),   nullptr)},
    {"get",            Command(_H(&CLIManager::_getParameterHelp),  _P(&CLIManager::_getParameter),  _C(&CLIManager::_getParameterCompletion))},
    {"set",            Command(_H(&CLIManager::_setParameterHelp),  _P(&CLIManager::_setParameter),  _C(&CLIManager::_setParameterCompletion))},
    {"save",           Command(_H(&CLIManager::_saveHelp),          _P(&CLIManager::_save),          _C(&CLIManager::_saveCompletion))},
    {"vxltoraw",       Command(_H(&CLIManager::_vxlToRawHelp),      _P(&CLIManager::_vxlToRaw),      _C(&CLIManager::_vxlToRawCompletion))},
    {"cap",            Command(_H(&CLIManager::_capabilitiesHelp),  _P(&CLIManager::_capabilities),  _C(&CLIManager::_capabilitiesCompletion))},
    {"help",           Command(_H(&CLIManager::_helpHelp),          _P(&CLIManager::_help),          nullptr)},
    {"disconnect",     Command(_H(&CLIManager::_disconnectHelp),    _P(&CLIManager::_disconnect),    nullptr)},
    {"reset",          Command(_H(&CLIManager::_resetHelp),         _P(&CLIManager::_reset),         nullptr)},
    {"exit",           Command(_H(&CLIManager::_exitHelp),          _P(&CLIManager::_exit),          nullptr)},
    {"filters",        Command(_H(&CLIManager::_filtersHelp),       _P(&CLIManager::_filters),       nullptr)},
    {"addfilter",      Command(_H(&CLIManager::_addFilterHelp),     _P(&CLIManager::_addFilter),     _C(&CLIManager::_addFilterCompletion))},
    {"removefilter",   Command(_H(&CLIManager::_removeFilterHelp),  _P(&CLIManager::_removeFilter),  _C(&CLIManager::_removeFilterCompletion))},
    {"setfilterparam", Command(_H(&CLIManager::_setFilterParamHelp),_P(&CLIManager::_setFilterParam),_C(&CLIManager::_setFilterParamCompletion))},
    
  });
  
  _specialParameters = Map<String, Command>({
    {"roi",       Command(_H(&CLIManager::_roiCapabilities),         _P(&CLIManager::_roi),         nullptr)},
    {"video_mode",Command(_H(&CLIManager::_videoModeCapabilities),   _P(&CLIManager::_videoMode),   nullptr)},
  });
  
  // Scan and connect to the first device available
  Vector<DevicePtr> devices = sys.scan();
  
  if(devices.size() > 0)
    _currentDepthCamera = sys.connect(devices[0]);
  
  if(_currentDepthCamera)
  {
    _viewer = Ptr<PCLViewer>(new PCLViewer());
    _viewer->setDepthCamera(_currentDepthCamera);
    _viewer->start();
  }
}

void CLIManager::run()
{
  _commandLoop();
}

void CLIManager::_getPrompt(String &prompt)
{
  if(_currentDepthCamera)
    prompt = "voxel:" + _currentDepthCamera->id() + "> ";
  else
    prompt = "voxel:> ";
}


void CLIManager::_commandLoop()
{
  _keepRunning = true;
  
  /* Load history from file. The history file is just a plain text file
   * where entries are separated by newlines. */
  linenoiseHistorySetMaxLen(1000);
  linenoiseHistoryLoad("history.txt"); /* Load the history at startup */
  
  linenoiseSetCompletionCallback(std::bind(&CLIManager::_completionCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  
  char *line;
  Vector<String> tokens;
  
  String prompt;
  
  _getPrompt(prompt);
  
  /* Now this is the main loop of the typical linenoise-based application.
   * The call to linenoise() will block as long as the user types something
   * and presses enter.
   *
   * The typed string is returned as a malloc() allocated string by
   * linenoise, so the user needs to free() it. */
  while(_keepRunning && (line = linenoise(prompt.c_str())) != NULL) 
  {
    /* Do something with the string. */
    if (line[0] != '\0') 
    {
      linenoiseHistoryAdd(line); /* Add to the history. */
      linenoiseHistorySave("history.txt"); /* Save the history on disk. */
      
      _getTokens(line, tokens);
      free(line);
      
      if(tokens.size() == 0)
      {
        logger(LOG_ERROR) << "Seems like a parse error. Please re-enter your command" << std::endl;
        continue;
      }
      
      auto c = _commands.find(tokens[0]);
      
      if(c == _commands.end())
      {
        logger(LOG_ERROR) << "Unknown command '" << tokens[0] << "'" << std::endl;
        continue;
      }
      
      if(!c->second.process)
      {
        logger(LOG_ERROR) << "Don't know how to execute command '" << tokens[0] << "'" << std::endl;
        continue;
      }
      
      c->second.process(tokens);
      _getPrompt(prompt);
      
      continue;
    }
    free(line);
  }
}

void CLIManager::_getTokens(const char *command, Vector<String> &tokens)
{
  const char *c = command;
  
  const char *previousEnd = command;
  
  bool inQuote = false;
  
  tokens.clear();
  
  auto index = 0;
  
  auto maxLength = strlen(command);
  
  while(*c != '\0' && index < maxLength)
  {
    index++;
    if(*c == ' ' || *c == '\t')
    {
      if(!inQuote)
      {
        if(previousEnd != c)
        {
          tokens.push_back(String(previousEnd, (size_t)(c - previousEnd)));
          previousEnd = c;
        }
        previousEnd++;
      }
      
      c++;
    }
    else if(isalnum(*c) || *c == '_' || *c == '.' || *c == '-' || *c == ':')
    {
      c++;
    }
    else if(*c == '"')
    {
      if(previousEnd != c)
      {
        tokens.push_back(String(previousEnd, (size_t)(c - previousEnd)));
        previousEnd = c;
      }
      else if(inQuote)
      {
        tokens.push_back(String()); // blank string for blank quote
      }
      
      c++;
      previousEnd = c;
      
      inQuote = !inQuote;
    }
    else if(!inQuote) // treat each non-alnum as a separate token
    {
      if(previousEnd != c)
      {
        tokens.push_back(String(previousEnd, (size_t)(c - previousEnd)));
        previousEnd = c;
      }
      
      tokens.push_back(String(c, 1));
      c++;
      previousEnd = c;
    }
    else // inQuote and non-alphabetic character
      c++;
  }
  
  if(previousEnd != c)
    tokens.push_back(String(previousEnd, (size_t)(c - previousEnd)));
}


void CLIManager::_helpHelp()          { std::cout << "help [cmd]                Print this help and for any specified Voxel command 'cmd'" << std::endl; }
void CLIManager::_exitHelp()          { std::cout << "exit                      Exit VoxelCLI" << std::endl; }
void CLIManager::_connectHelp()       { std::cout << "connect <dev>             Connect to a specific device. <dev> needs to be device ID\n"
                                                  << "                          in the format INTERFACE::DEVICE::SERIAL. Use double-quotes for the ID if it contains spaces" << std::endl; }
void CLIManager::_listHelp()          { std::cout << "list                      Scan and list all valid Voxel connectable devices" << std::endl; }
void CLIManager::_currentHelp()       { std::cout << "status                    Show the current connected device ID if present" << std::endl; }
void CLIManager::_startHelp()         { std::cout << "start                     Start streaming and showing the current device" << std::endl; }
void CLIManager::_stopHelp()          { std::cout << "stop                      Stop streaming the current device" << std::endl; }
void CLIManager::_getRegisterHelp()   { std::cout << "getr <reg>                Get register value at <reg>. Use '0x' prefix for hexadecimal" << std::endl; }
void CLIManager::_setRegisterHelp()   { std::cout << "setr <reg> = <value>      Set register <reg> to <value>. Use '0x' prefix for hexadecimal" << std::endl; }
void CLIManager::_getParameterHelp()  { std::cout << "get <param>               Get parameter value given by name <param>" << std::endl; }
void CLIManager::_capabilitiesHelp()  { std::cout << "cap [<param>][*]          Get capabilities of the current depth camera.\n"
                                                  << "                          Optionally a parameter name can be given to list only that parameter details given by name <param>.\n"
                                                  << "                          A optional wildcard can be given to list all parameters beginning with name <param>" << std::endl; }
void CLIManager::_filtersHelp()       { std::cout << "filters                   List available filters and currently in use filters" << std::endl; }
void CLIManager::_addFilterHelp()     { std::cout << "addfilter <frametype> <name> <pos>    Add filter <name> for <frametype> at <pos>.\n"
                                                  << "                          Set <pos> = -1 to add the end.\n"
                                                  << "                          <frametype> can be raw/raw_processed/depth."<< std::endl; }
void CLIManager::_removeFilterHelp()  { std::cout << "removefilter <frametype> <pos>    Remove filter with id <filterid> for <frametype>.\n"
                                                  << "                          <frametype> can be raw/raw_processed/depth."<< std::endl; }
void CLIManager::_setFilterParamHelp(){ std::cout << "setfilterparam <frametype> <filterid> <param> = <value>   Set parameter <param> to <value> for filter\n"
                                                  << "                          with id <filterid> present for frame type <frametype>.\n"
                                                  << "                          <frametype> can be raw/raw_processed/depth" << std::endl; }
void CLIManager::_setParameterHelp()  { std::cout << "set <param> = <value>     Set parameter value given by name <param>. Use '0x' prefix for hexadecimal" << std::endl; }
void CLIManager::_saveHelp()          { std::cout << "save type count filename  Save current 'count' number of frames.\n"
                                                  << "                          'type' = raw/phase/ambient/amplitude/flags/depth/pointcloud/vxl" << std::endl; }
void CLIManager::_vxlToRawHelp()      { std::cout << "vxltoraw rawtype vxlfile rawfile   Get raw data from saved VXL file.\n"
                                                  << "                          'rawtype' = raw/phase/ambient/amplitude/flags/depth/pointcloud" << std::endl; }                                                  
void CLIManager::_disconnectHelp()    { std::cout << "disconnect                Disconnect the currently connected depth camera" << std::endl; }
void CLIManager::_resetHelp()         { std::cout << "reset                     Reset and disconnect the currently connected depth camera" << std::endl; }


void CLIManager::_help(const Vector<String> &tokens)
{
  if(tokens.size() >= 2)
  {
    auto c = _commands.find(tokens[1]);
    
    if(c == _commands.end())
    {
      logger(LOG_ERROR) << "Unknown command '" << tokens[1] << "'" << std::endl;
      return;
    }
    
    if(!c->second.help)
    {
      logger(LOG_ERROR) << "No help available about command '" << tokens[1] << "'" << std::endl;
      return;
    }
    
    c->second.help();
  }
  else
  {
    for(auto &c: _commands)
      if(c.second.help)
      {
        c.second.help();
        std::cout << std::endl;
      }
  }
}

void CLIManager::_exit(const Vector<String> &tokens)
{
  _keepRunning = false;
  _disconnect(tokens);
}

void CLIManager::_list(const Vector<String> &tokens)
{
  Vector<DevicePtr> devices = _sys.scan();
  
  if(!devices.size())
  {
    logger(LOG_ERROR) << "Could not find any valid devices." << std::endl;
    return;
  }
  
  for(auto &d: devices)
  {
    std::cout << d->id();
    
    if(d->description().size())
      std::cout << " -- " << d->description();
    
    std::cout << std::endl;
  }
}

void CLIManager::_connect(const Vector<String> &tokens)
{
  if(tokens.size() < 2)
  {
    logger(LOG_ERROR) << "Please specify a device ID" << std::endl;
    return;
  }
  
  std::cout << "Searching for '" << tokens[1] << "'..." << std::endl;
  
  Vector<DevicePtr> devices = _sys.scan();
  
  for(auto &d: devices)
  {
    if(d->id() == tokens[1])
    {
      DepthCameraPtr p = _sys.connect(d);
      
      if(!p)
      {
        std::cout << "Couldn't connect to specific device ID" << std::endl;
        return;
      }
      
      bool wasRunning = false;
      if(_viewer && _viewer->isRunning())
      {
        _stop(tokens);
        wasRunning = true;
      }
      
      if(_currentDepthCamera)
      {
        _disconnect(tokens); // disconnect currently connected depth camera
      }
      
      _currentDepthCamera = p;
      
      if(wasRunning)
        _start(tokens);
      
      return;
    }
  }
  
  logger(LOG_ERROR) << "Could not find a valid device with specified ID" << std::endl;
}

void CLIManager::_current(const Vector<String> &tokens)
{
  if(_currentDepthCamera)
  {
    std::cout << "Current device ID = " << _currentDepthCamera->id() << std::endl;
    
    if(_viewer && _viewer->isRunning())
      std::cout << "It is currently streaming. Use 'stop' command to stop streaming" << std::endl;
    else
      std::cout << "It is currently not streaming. Use 'start' command to start streaming" << std::endl;
  }
  else
    std::cout << "No device is current in use by this CLI" << std::endl;
}

void CLIManager::_start(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No device connected. Kindly 'connect' a device first" << std::endl;
    return;
  }
  
  if(!_viewer)
  {
    _viewer = Ptr<PCLViewer>(new PCLViewer());
  }
  
  if(!_viewer->isRunning())
  {
    _viewer->setDepthCamera(_currentDepthCamera);
    _viewer->start();
  }
  else
  {
    _viewer->stop();
    _viewer->setDepthCamera(_currentDepthCamera);
    _viewer->start();
  }
}

void CLIManager::_stop(const Vector<String> &tokens)
{
  if(!_viewer)
    return;
  
   _viewer->stop();
}

void CLIManager::_getParameter(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "Please connect to a depth camera first" << std::endl;
    return;
  }
  
  if(tokens.size() < 2)
  {
    logger(LOG_ERROR) << "Please specific a parameter name" << std::endl;
    return;
  }
  
  auto sp = _specialParameters.find(tokens[1]);
  
  // Is a special parameter?
  if(sp != _specialParameters.end() && sp->second.process)
  {
    sp->second.process(tokens);
    return;
  }
  
  ParameterPtr param = _currentDepthCamera->getParam(tokens[1]);
  
  if(!param)
  {
    logger(LOG_ERROR) << "No valid parameter with name = '" << tokens[1] << "'" << std::endl;
    return;
  }
  
  BoolParameter *boolParam = dynamic_cast<BoolParameter *>(param.get());
  IntegerParameter *intParam = dynamic_cast<IntegerParameter *>(param.get());
  UnsignedIntegerParameter *uintParam = dynamic_cast<UnsignedIntegerParameter *>(param.get());
  FloatParameter *floatParam = dynamic_cast<FloatParameter *>(param.get());
  EnumParameter *enumParam = dynamic_cast<EnumParameter *>(param.get());
  
  if(boolParam)
  {
    bool value;
    if(!boolParam->get(value))
    {
      logger(LOG_ERROR) << "Failed to get parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << tokens[1] << " = " << (value?"true":"false");
    
    const Vector<String> &meaning = boolParam->valueMeaning();
    
    if(meaning.size() == 2 && meaning[value].size())
      std::cout << " (" << meaning[value] << ")";
      
    std::cout << std::endl;
      
    return;
  }
  
  if(intParam)
  {
    int value;
    if(!intParam->get(value))
    {
      logger(LOG_ERROR) << "Failed to get parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << tokens[1] << " = " << std::dec << value  << " " << intParam->unit() << std::endl;
    
    return;
  }
  
  if(uintParam)
  {
    uint value;
    if(!uintParam->get(value))
    {
      logger(LOG_ERROR) << "Failed to get parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << tokens[1] << " = " << std::dec << value << " " << uintParam->unit() << std::endl;
    
    return;
  }
  
  if(floatParam)
  {
    float value;
    if(!floatParam->get(value))
    {
      logger(LOG_ERROR) << "Failed to get parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << tokens[1] << " = " << std::dec << value << " " << floatParam->unit() << std::endl;
    
    return;
  }
  
  if(enumParam)
  {
    int value;
    if(!enumParam->get(value))
    {
      logger(LOG_ERROR) << "Failed to get parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << tokens[1] << " = "  << std::dec << value;
    
    const Vector<String> &meaning = enumParam->valueMeaning();
    
    if(meaning.size() > value && meaning[value].size())
      std::cout << " (" << meaning[value] << ")";
    
    std::cout << std::endl;
    return;
  }
  
  logger(LOG_ERROR) << "Unknown type of parameter '" << tokens[1] << "'. Don't know how to handle" << std::endl;
}

void CLIManager::_setParameter(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "Please connect to a depth camera first" << std::endl;
    return;
  }
  
  if(tokens.size() < 4 || tokens[2] != "=")
  {
    logger(LOG_ERROR) << "Please specify a parameter name and value to be set in the format given below." << std::endl;
    _setParameterHelp();
    return;
  }
  
  auto sp = _specialParameters.find(tokens[1]);
  
  // Is a special parameter?
  if(sp != _specialParameters.end() && sp->second.process)
  {
    sp->second.process(tokens);
    return;
  }
  
  ParameterPtr param = _currentDepthCamera->getParam(tokens[1]);
  
  if(!param)
  {
    logger(LOG_ERROR) << "No valid parameter with name = '" << tokens[1] << "'" << std::endl;
    return;
  }
  
  BoolParameter *boolParam = dynamic_cast<BoolParameter *>(param.get());
  IntegerParameter *intParam = dynamic_cast<IntegerParameter *>(param.get());
  UnsignedIntegerParameter *uintParam = dynamic_cast<UnsignedIntegerParameter *>(param.get());
  FloatParameter *floatParam = dynamic_cast<FloatParameter *>(param.get());
  EnumParameter *enumParam = dynamic_cast<EnumParameter *>(param.get());
  
  if(boolParam)
  {
    bool value;
    
    if(tokens[3] == "true")
      value = true;
    else if(tokens[3] == "false")
      value = false;
    else
    {
      std::istringstream s(tokens[3]);
      s.unsetf(std::ios_base::basefield);
      s >> value;
    }
    
    std::cout << "Setting parameter '" << tokens[1] << "' = " << (value?"true":"false") << " ..." << std::endl;
    
    if(!boolParam->set(value))
    {
      logger(LOG_ERROR) << "Failed to set parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << "Successfully set parameter '" << tokens[1] << "'" << std::endl;
    
    return;
  }
  
  if(intParam)
  {
    int value;
    
    std::istringstream s(tokens[3]);
    s.unsetf(std::ios_base::basefield);
    s >> value;
    
    std::cout << "Setting parameter '" << tokens[1] << "' = " << std::dec << value << " " << intParam->unit() << " ..." << std::endl;
    
    if(!intParam->set(value))
    {
      logger(LOG_ERROR) << "Failed to set parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << "Successfully set parameter '" << tokens[1] << "'" << std::endl;
    
    return;
  }
  
  if(uintParam)
  {
    uint value;
    
    std::istringstream s(tokens[3]);
    s.unsetf(std::ios_base::basefield);
    s >> value;
    
    std::cout << "Setting parameter '" << tokens[1] << "' = " << std::dec << value << " " << uintParam->unit() << " ..." << std::endl;
    
    if(!uintParam->set(value))
    {
      logger(LOG_ERROR) << "Failed to set parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << "Successfully set parameter '" << tokens[1] << "'" << std::endl;
    
    return;
  }
  
  if(floatParam)
  {
    float value;
    
    std::istringstream s(tokens[3]);
    s >> value;
    
    std::cout << "Setting parameter '" << tokens[1] << "' = " << std::dec << value << " " << floatParam->unit() << " ..." << std::endl;
    
    if(!floatParam->set(value))
    {
      logger(LOG_ERROR) << "Failed to set parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << "Successfully set parameter '" << tokens[1] << "'" << std::endl;
    
    return;
  }
  
  if(enumParam)
  {
    int value;
    
    std::istringstream s(tokens[3]);
    s.unsetf(std::ios_base::basefield);
    s >> value;
    
    std::cout << "Setting parameter '" << tokens[1] << "' = " << std::dec << value << " ..." << std::endl;
    
    if(!enumParam->set(value))
    {
      logger(LOG_ERROR) << "Failed to set parameter '" << tokens[1] << "'" << std::endl;
      return;
    }
    std::cout << "Successfully set parameter '" << tokens[1] << "'" << std::endl;
    
    return;
  }
  
  logger(LOG_ERROR) << "Unknown type of parameter '" << tokens[1] << "'. Don't know how to handle" << std::endl;
}

void CLIManager::_getRegister(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "Please connect to a depth camera first" << std::endl;
    return;
  }
  
  if(!_currentDepthCamera->getProgrammer())
  {
    logger(LOG_ERROR) << "Current depth camera " << _currentDepthCamera-> id() << " does not have a register programmer." << std::endl;
    return;
  }
  
  if(tokens.size() < 2)
  {
    logger(LOG_ERROR) << "Please specify a register address to read from" << std::endl;
    return;
  }
  
  uint address, value;
  
  std::istringstream s(tokens[1]);
  s.unsetf(std::ios_base::basefield);
  s >> address;
  
  if(!_currentDepthCamera->getProgrammer()->readRegister(address, value))
  {
    logger(LOG_ERROR) << "Could not read register @0x" << std::hex << address << std::endl;
  }
  else
    std::cout << "Register @0x" << std::hex << address << " = 0x" << std::hex << value << std::endl;
}

void CLIManager::_setRegister(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "Please connect to a depth camera first" << std::endl;
    return;
  }
  
  if(!_currentDepthCamera->getProgrammer())
  {
    logger(LOG_ERROR) << "Current depth camera " << _currentDepthCamera-> id() << " does not have a register programmer." << std::endl;
    return;
  }
  
  if(tokens.size() < 4 || tokens[2] != "=")
  {
    logger(LOG_ERROR) << "Please specify a register address to write to and value to set in the following format" << std::endl;
    _setRegisterHelp();
    return;
  }
  
  uint address, value;
  
  {
    std::istringstream s(tokens[1]);
    s.unsetf(std::ios_base::basefield);
    s >> address;
  }
  
  {
    std::istringstream s(tokens[3]);
    s.unsetf(std::ios_base::basefield);
    s >> value;
  }
  
  std::cout << "Setting register @0x" << std::hex << address << " = " << std::hex << "0x" << value << " ..." << std::endl;
  
  if(!_currentDepthCamera->getProgrammer()->writeRegister(address, value))
  {
    logger(LOG_ERROR) << "Could not write to register @0x" << std::hex << address << std::endl;
  }
  else
    std::cout << "Successfully set register @0x" << std::hex << address << std::endl;
}

void CLIManager::_capabilities(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "Please connect to a depth camera first" << std::endl;
    return;
  }
  
  if(tokens.size() < 2)
  {
    // Show special parameters first
    for(auto &sp: _specialParameters)
    {
      if(sp.second.help)
        sp.second.help();
    }
    
    const Map<String, ParameterPtr> &parameters = _currentDepthCamera->getParameters();
    for(auto &p: parameters)
    {
      _showParameterInfo(p.second);
    }
  }
  else if(tokens.size() == 2)
  {
    auto sp = _specialParameters.find(tokens[1]);
    
    // Is a special parameter?
    if(sp != _specialParameters.end() && sp->second.help)
    {
      sp->second.help();
      return;
    }
    
    _showParameterInfo(_currentDepthCamera->getParam(tokens[1]));
  }
  else if(tokens[2] == "*") // wildcard match?
  {
    for(auto &sp: _specialParameters)
    {
      const String &n = sp.first;
      if(n.size() < tokens[1].size()) // parameter name smaller than search name?
        continue;
      
      if(n.compare(0, tokens[1].size(), tokens[1]) == 0) // matches first part in the name?
        sp.second.help();
    }
    
    const Map<String, ParameterPtr> &parameters = _currentDepthCamera->getParameters();
    for(auto &p: parameters)
    {
      const String &n = p.second->name();
      
      if(n.size() < tokens[1].size()) // parameter name smaller than search name?
        continue;
      
      if(n.compare(0, tokens[1].size(), tokens[1]) == 0) // matches first part in the name?
        _showParameterInfo(p.second);
    }
  }
}

void CLIManager::_showParameterInfo(const ParameterPtr &param)
{
  if(!param)
  {
    logger(LOG_ERROR) << "Null parameter given for display" << std::endl;
    return;
  }
  
  std::cout << std::endl;
  
  BoolParameter *boolParam = dynamic_cast<BoolParameter *>(param.get());
  IntegerParameter *intParam = dynamic_cast<IntegerParameter *>(param.get());
  UnsignedIntegerParameter *uintParam = dynamic_cast<UnsignedIntegerParameter *>(param.get());
  FloatParameter *floatParam = dynamic_cast<FloatParameter *>(param.get());
  EnumParameter *enumParam = dynamic_cast<EnumParameter *>(param.get());
  
  String n = param->name() + ((param->ioType() == Parameter::IO_READ_ONLY)?" [R]":" [RW]");
  std::cout << n;
  
  for(auto i = 0; i < 4 - n.size()/8; i++)
    std::cout << "\t";
  
  bool descShown = true;
  
  if(param->displayName().size())
    std::cout << "(" << param->displayName() << "). " << param->description() << std::endl;
  else if(param->description().size())
    std::cout << param->description() << std::endl;
  else
    descShown = false;
  
  
  if(param->address())
  {
    if(descShown)
      std::cout << "\t\t\t\t";
    std::cout << "Register: 0x" << std::hex << param->address() << "[" << std::dec << (uint)param->msb() << ":" << (uint)param->lsb() << "]" << std::endl;
  }
  
  if(boolParam) // FIXME: Handle strobe type later
  {
    bool value;
    if(boolParam->get(value))
      std::cout << "\t\t\t\tCurrent value: " << (value?"true":"false") << std::endl;
    if(boolParam->valueMeaning().size())
    {
      std::cout << "\t\t\t\tPossible values:" << std::endl;
      
      for(auto i = 0; i < boolParam->valueMeaning().size(); i++)
      {
        std::cout << "\t\t\t\t" << ((i == 0)?"false":"true");
        
        if(boolParam->valueMeaning()[i].size())
          std::cout << " -> " << boolParam->valueMeaning()[i];
        
        if(boolParam->valueDescription().size() > i &&boolParam->valueDescription()[i].size())
        {
          std::cout << " (" << boolParam->valueDescription()[i] << ")";
        }
        
        std::cout << std::endl;
      }
    }
    
    return;
  }
  
  if(intParam)
  {
    int value;
    if(intParam->get(value))
      std::cout << "\t\t\t\tCurrent value: " << std::dec << value << " " << intParam->unit() << std::endl;
    
    std::cout << "\t\t\t\tLimits: [" << std::dec << intParam->lowerLimit() << " " << intParam->unit() << ", " << intParam->upperLimit() << " " << intParam->unit() << "]" << std::endl;
    
    return;
  }
  
  if(uintParam)
  {
    uint value;
    if(uintParam->get(value))
      std::cout << "\t\t\t\tCurrent value: " << std::dec << value << " " << uintParam->unit() << std::endl;
    
    std::cout << "\t\t\t\tLimits: [" << std::dec << uintParam->lowerLimit() << " " << uintParam->unit() << ", " << uintParam->upperLimit() << " " << uintParam->unit() << "]" << std::endl;
    
    return;
  }
  
  if(floatParam)
  {
    float value;
    if(floatParam->get(value))
      std::cout << "\t\t\t\tCurrent value: " << std::dec << value << " " << floatParam->unit() << std::endl;
    
    std::cout << "\t\t\t\tLimits: [" << std::dec << floatParam->lowerLimit() << " " << floatParam->unit() << ", " << floatParam->upperLimit() << " " << floatParam->unit() << "]" << std::endl;
    
    return;
  }
  
  if(enumParam)
  {
    int value;
    if(enumParam->get(value))
      std::cout << "\t\t\t\tCurrent value: " << std::dec << value << std::endl;
    if(enumParam->allowedValues().size())
    {
      std::cout << "\t\t\t\tPossible values:" << std::endl;
      
      for(auto i = 0; i < enumParam->allowedValues().size(); i++)
      {
        std::cout << "\t\t\t\t" << std::dec << enumParam->allowedValues()[i];
        
        if(enumParam->valueMeaning().size() > i &&enumParam->valueMeaning()[i].size())
          std::cout << " -> " << enumParam->valueMeaning()[i];
        
        if(enumParam->valueDescription().size() > i &&enumParam->valueDescription()[i].size())
        {
          std::cout << " (" << enumParam->valueDescription()[i] << ")";
        }
        
        std::cout << std::endl;
      }
    }
    
    return;
  }
  
}

void CLIManager::_completionCallback(const char *buf, linenoiseCompletions *lc)
{
  Vector<String> tokens;
  
  _getTokens(buf, tokens);
  
  if(tokens.size() == 0)
    return;
  
  uint length = strlen(buf);
  
  if(tokens.size() > 1 || buf[length - 1] == ' ' || buf[length - 1] == '\t') // seems like one complete command has been given
  {
    auto c = _commands.find(tokens[0]);
    
    if(c == _commands.end())
      return;
    
    if(c->second.complete)
      c->second.complete(tokens, lc);
  }
  else // still typing the main command itself...
  {
    for(auto &c: _commands)
    {
      if(c.first.size() >= tokens[0].size() && c.first.compare(0, tokens[0].size(), tokens[0]) == 0)
        linenoiseAddCompletion(lc, c.first.c_str());
    }
  }
}

void CLIManager::_connectCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(tokens.size() == 2)
  {
    Vector<DevicePtr> devices = _sys.scan();
    
    for(auto &d: devices)
    {
      if(d->id().size() >= tokens[1].size() && d->id().compare(0, tokens[1].size(), tokens[1]) == 0)
        linenoiseAddCompletion(lc, (tokens[0] + " \"" + d->id() + "\"").c_str());
    }   
  }
  else if(tokens.size() == 1)
  {
    Vector<DevicePtr> devices = _sys.scan();
    
    for(auto &d: devices)
    {
      linenoiseAddCompletion(lc, (tokens[0] + " \"" + d->id() + "\"").c_str());
    }
  }
}

void CLIManager::_paramCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(!_currentDepthCamera)
    return;
  
  uint count = 0;
  if(tokens.size() == 2)
  {
    for(auto &sp: _specialParameters)
    {
      if(sp.first.size() >= tokens[1].size() && sp.first.compare(0, tokens[1].size(), tokens[1]) == 0)
      {
        linenoiseAddCompletion(lc, (tokens[0] + " " + sp.first).c_str());
        count++;
      }
    }
    
    for(auto &p: _currentDepthCamera->getParameters())
    {
      if(p.first.size() >= tokens[1].size() && p.first.compare(0, tokens[1].size(), tokens[1]) == 0)
      {
        linenoiseAddCompletion(lc, (tokens[0] + " " + p.first).c_str());
        count++;
        
        if(count >= 100) // return a maximum of 100 matches
          return;
      }
    }   
  }
  else if(tokens.size() == 1)
  {
    for(auto &sp: _specialParameters)
    {
      linenoiseAddCompletion(lc, (tokens[0] + " " + sp.first).c_str());
      count++;
    }
    
    for(auto &p: _currentDepthCamera->getParameters())
    {
      linenoiseAddCompletion(lc, (tokens[0] + " " + p.first).c_str());
      count++;
        
      if(count >= 100) // return a maximum of 100 matches
        return;
    }
  }
}

void CLIManager::_capabilitiesCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  _paramCompletion(tokens, lc);
}


void CLIManager::_getParameterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  _paramCompletion(tokens, lc);
}

void CLIManager::_setParameterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  _paramCompletion(tokens, lc);
}

template <>
bool CLIManager::_saveFrameToFile<RawDataFrame>(const Frame *frame, OutputFileStream &saveFile, const String &subType)
{
  const RawDataFrame *f = dynamic_cast<const RawDataFrame *>(frame);
  
  if(!f)
  {
    logger(LOG_ERROR) << "Null frame captured? or not of type RawDataFrame" << std::endl;
    return false;
  }
  
  std::cout << "Saving frame " << f->id << "@" << f->timestamp << "us";
  
  if(_lastTimeStamp != 0)
    std::cout << " (" << 1E6/(f->timestamp - _lastTimeStamp) << " fps)";
  
  std::cout << ", raw frame size = " << f->data.size() << "bytes" << std::endl;
  
  _lastTimeStamp = f->timestamp;
  _saveFile.write((char *)f->data.data(), f->data.size());
  
  return true;
}

template <>
bool CLIManager::_saveFrameToFile<ToFRawFrame>(const Frame *frame, OutputFileStream &saveFile, const String &subType)
{
  const ToFRawFrame  *f = dynamic_cast<const ToFRawFrame *>(frame);
  
  if(!f)
  {
    logger(LOG_ERROR) << "Null frame captured? or not of type RawDataFrame" << std::endl;
    return false;
  }
  
  std::cout << "Saving frame " << f->id << "@" << f->timestamp << "us";
  
  if(_lastTimeStamp != 0)
    std::cout << " (" << 1E6/(f->timestamp - _lastTimeStamp) << " fps)";
  
  std::cout << ", " << subType << " frame size = " << f->size.width << "x" << f->size.height << " (row-ordered";
  
  _lastTimeStamp = f->timestamp;
  
  if(f->phase() && subType == "phase")
  {
    std::cout << ", " << f->phaseWordWidth() << "byte(s) per pixel)" << std::endl;
    _saveFile.write((char *)f->phase(), f->phaseWordWidth()*f->size.width * f->size.height);
  }
  
  if(f->amplitude() && subType == "amplitude")
  {
    std::cout << ", " << f->amplitudeWordWidth() << "byte(s) per pixel)" << std::endl;
    _saveFile.write((char *)f->amplitude(), f->amplitudeWordWidth()*f->size.width * f->size.height);
  }
  
  if(f->ambient() && subType == "ambient")
  {
    std::cout << ", " << f->ambientWordWidth() << "byte(s) per pixel)" << std::endl;
    _saveFile.write((char *)f->ambient(), f->ambientWordWidth()*f->size.width * f->size.height);
  }
  
  if(f->flags() && subType == "flags")
  {
    std::cout << ", " << f->flagsWordWidth() << "byte(s) per pixel)" << std::endl;
    _saveFile.write((char *)f->flags(), f->flagsWordWidth()*f->size.width * f->size.height);      
  }
  
  return true;
}

template <>
bool CLIManager::_saveFrameToFile<DepthFrame>(const Frame *frame, OutputFileStream &saveFile, const String &subType)
{
  const DepthFrame *f = dynamic_cast<const DepthFrame *>(frame);
  
  if(!f)
  {
    logger(LOG_ERROR) << "Null frame captured? or not of type RawDataFrame" << std::endl;
    return false;
  }
  
  std::cout << "Saving frame " << f->id << "@" << f->timestamp << "us";
  
  if(_lastTimeStamp != 0)
    std::cout << " (" << 1E6/(f->timestamp - _lastTimeStamp) << " fps)";
  
  std::cout << ", frame size = " << f->size.width << "x" << f->size.height << ", 4-bytes (float) per pixel" << std::endl;
  
  _lastTimeStamp = f->timestamp;
  
  _saveFile.write((char *)f->depth.data(), sizeof(float)*f->size.width*f->size.height);
  _saveFile.write((char *)f->amplitude.data(), sizeof(float)*f->size.width*f->size.height);
  
  return true;
}

template <>
bool CLIManager::_saveFrameToFile<XYZIPointCloudFrame>(const Frame *frame, OutputFileStream &saveFile, const String &subType)
{
  const XYZIPointCloudFrame *f = dynamic_cast<const XYZIPointCloudFrame *>(frame);
  
  if(!f)
  {
    logger(LOG_ERROR) << "Null frame captured? or not of type RawDataFrame" << std::endl;
    return false;
  }
  
  std::cout << "Saving frame " << f->id << "@" << f->timestamp << "us";
  
  if(_lastTimeStamp != 0)
    std::cout << " (" << 1E6/(f->timestamp - _lastTimeStamp) << " fps)";
  
  std::cout << ", number of points = " << f->points.size() << ", 4-bytes float (x, y, z, i) data" << std::endl;
  
  _lastTimeStamp = f->timestamp;
  _saveFile.write((char *)f->points.data(), sizeof(IntensityPoint)*f->points.size());
  
  return true;
}


void CLIManager::_save(const Vector<String> &tokens)
{
  if(!_currentDepthCamera || !_viewer->isRunning())
  {
    logger(LOG_ERROR) << "Cannot save without an active depth camera. Please make sure a depth camera is connect and is streaming. See 'status' to find this out." << std::endl;
    return;
  }
  
  if(tokens.size() < 4)
  {
    logger(LOG_ERROR) << "Required parameters missing." << std::endl;
    _saveHelp();
    return;
  }
  
  _count = atoi(tokens[2].c_str());
  
  if(_count > 100000 || _count <= 0)
  {
    logger(LOG_ERROR) << "Invalid count. Allowed values are 1 to 100000" << std::endl;
    _saveHelp();
    return;
  }
  
  _currentCount = 0;
  _lastTimeStamp = 0;
  
  if(tokens[1] != "vxl")
  {
    _saveFile.close();
    _saveFile.clear();
    _saveFile.open(tokens[3], std::ios::out | std::ios::binary);
    
    if(!_saveFile.good())
    {
      logger(LOG_ERROR) << "Could not open '" << tokens[3] << "' for writing frames." << std::endl;
      return;
    }
  }
  
  _saveFileName = tokens[3];
  
  if(tokens[1] == "raw")
  {
    _saveCallbackConnection = _viewer->getGrabber()->registerCallback<PCLGrabber::RawImageCallBack>([this](const Voxel::RawFrame &frame, const Voxel::DepthCamera::FrameType type) -> void {
      if(type != DepthCamera::FRAME_RAW_FRAME_UNPROCESSED)
        return;
      
      if(!_saveFrameToFile<RawDataFrame>(&frame, _saveFile))
      {
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
      
      _currentCount++;
      
      if(_currentCount >= _count)
      {
        std::cout << "Saved to file '" << _saveFileName << "'" << std::endl;
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
    });
  } 
  else if(tokens[1] == "phase" || tokens[1] == "amplitude" || tokens[1] == "ambient" || tokens[1] == "flags")
  {
    _saveCallbackConnection = _viewer->getGrabber()->registerCallback<PCLGrabber::RawImageCallBack>([this, &tokens](const Voxel::RawFrame &frame, const Voxel::DepthCamera::FrameType type) {
      if(type != DepthCamera::FRAME_RAW_FRAME_PROCESSED)
        return;
      
      if(!_saveFrameToFile<ToFRawFrame>(&frame, _saveFile, tokens[1]))
      {
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
      
      _currentCount++;
      
      if(_currentCount >= _count)
      {
        std::cout << "Saved to file '" << _saveFileName << "'" << std::endl;
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
    });
  } 
  else if(tokens[1] == "depth")
  {
    _saveCallbackConnection = _viewer->getGrabber()->registerCallback<PCLGrabber::DepthImageCallBack>([this](const Voxel::DepthFrame &frame) {
      
      if(!_saveFrameToFile<DepthFrame>(&frame, _saveFile))
      {
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
      
      _currentCount++;
      
      if(_currentCount >= _count)
      {
        std::cout << "Saved to file '" << _saveFileName << "'" << std::endl;
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
    });
  } 
  else if(tokens[1] == "pointcloud")
  {
    _saveCallbackConnection = _viewer->getGrabber()->registerCallback<PCLGrabber::PointCloudFrameCallBack>([this](const Voxel::PointCloudFrame &frame) {
      
      if(!_saveFrameToFile<XYZIPointCloudFrame>(&frame, _saveFile))
      {
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
      
      _currentCount++;
      
      if(_currentCount >= _count)
      {
        std::cout << "Saved to file '" << _saveFileName << "'" << std::endl;
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
    });
  }
  else if(tokens[1] == "vxl")
  {
    _saveCallbackConnection = _viewer->getGrabber()->registerCallback<PCLGrabber::RawImageCallBack>([this](const Voxel::RawFrame &frame, const Voxel::DepthCamera::FrameType type) -> void {
      if(type != DepthCamera::FRAME_RAW_FRAME_UNPROCESSED)
        return;
      
      const RawDataFrame *d = dynamic_cast<const RawDataFrame *>(&frame);
      
      if(!d)
      {
        logger(LOG_ERROR) << "Null frame captured? or not of type RawDataFrame" << std::endl;
        return;
      }
      
      std::cout << "Capture frame " << d->id << "@" << d->timestamp;
      
      if(_lastTimeStamp != 0)
        std::cout << " (" << 1E6/(d->timestamp - _lastTimeStamp) << " fps)";
      
      std::cout << std::endl;
      
      _lastTimeStamp = d->timestamp;
      
      if(_currentCount == 0 && !_currentDepthCamera->saveFrameStream(_saveFileName))
      {
        logger(LOG_ERROR) << "Failed to open '" << _saveFileName << "'" << std::endl;
        _currentDepthCamera->closeFrameStream();
        _saveCallbackConnection.disconnect();
        return;
      }
      
      if(_currentCount >= _count)
      {
        std::cout << "Saved to file '" << _saveFileName << "'" << std::endl;
        _currentDepthCamera->closeFrameStream();
        _saveCallbackConnection.disconnect();
      }
      
      _currentCount++;
    });
  }
  else
  {
    logger(LOG_ERROR) << "Unknown frame type '" << tokens[1] << "'" << std::endl;
    _saveFile.close();
    _saveHelp();
    return;
  }
  
  if(!_saveCallbackConnection.connected())
  {
    logger(LOG_ERROR) << "Could not connect to grabber to save the frames." << std::endl;
    _saveFile.close();
  }
}

void CLIManager::_saveCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(!_currentDepthCamera)
    return;
  
  if(tokens.size() == 1)
  {
    linenoiseAddCompletion(lc, (tokens[0] + " vxl").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " raw").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " phase").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " amplitude").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " ambient").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " flags").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " depth").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " pointcloud").c_str());
  }
  else if(tokens.size() == 2)
  {
    Vector<String> types = {"raw", "phase", "amplitude", "ambient", "flags", "depth", "pointcloud", "vxl"};
    
    for(auto &t: types)
    {
      if(t.size() < tokens[1].size())
        continue;
      
      if(t.compare(0, tokens[1].size(), tokens[1]) == 0)
        linenoiseAddCompletion(lc, (tokens[0] + " " + t).c_str());
    }
  }
}

void CLIManager::_disconnect(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No depth camera is current connected" << std::endl;
    return;
  }
  
  _stop(tokens);
  _viewer->removeDepthCamera();
  _sys.disconnect(_currentDepthCamera);
  _currentDepthCamera = nullptr;
}

void CLIManager::_roiCapabilities()
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No depth camera currently connected." << std::endl;
    return;
  }
  
  std::cout << "roi [RW]\t\t\t";
  
  breakLines("To set, use 'set roi = x,y,width,height'.", std::cout, 60, "\t\t\t\t");
  
  String message;
  if(_currentDepthCamera->allowedROI(message))
    breakLines(message, std::cout, 60, "\t\t\t\t");
  
  RegionOfInterest roi;
  
  if(_currentDepthCamera->getROI(roi))
  {
    std::cout << "Current value: [" << std::dec << roi.x << "," << roi.y << "," << roi.width << "," << roi.height << "]" << std::endl;
  }
}

void CLIManager::_roi(const Vector<String> &tokens)
{
  if(tokens.size() < 2)
  {
    logger(LOG_ERROR) << "Please specify roi parameter" << std::endl;
    return;
  }
  
  // get roi
  if(tokens[0] == "get")
  {
    RegionOfInterest roi;
    
    if(_currentDepthCamera->getROI(roi))
    {
      std::cout << "ROI = [" << std::dec << roi.x << "," << roi.y << "," << roi.width << "," << roi.height << "]" << std::endl;
    }
    else
    {
      logger(LOG_ERROR) << "Could not get ROI" << std::endl;
    }
  }
  else if(tokens[0] == "set")
  {
    if(tokens.size() < 10 || tokens[2] != "=")
    {
      logger(LOG_ERROR) << "Invalid syntax used for setting ROI" << std::endl;
      _roiCapabilities();
      return;
    }
    
    RegionOfInterest roi;
    
    char *endptr;
    
    roi.x = strtol(tokens[3].c_str(), &endptr, 10);
    roi.y = strtol(tokens[5].c_str(), &endptr, 10);
    roi.width = strtol(tokens[7].c_str(), &endptr, 10);
    roi.height = strtol(tokens[9].c_str(), &endptr, 10);
    
    if(!_currentDepthCamera->setROI(roi))
    {
      logger(LOG_ERROR) << "Could not set ROI" << std::endl;
    }
    else
      std::cout << "Successfully set ROI" << std::endl;
  }
  else
  {
    logger(LOG_ERROR) << "Unknown command '" << tokens[0] << "' used for ROI" << std::endl;
  }
}

void CLIManager::_videoModeCapabilities()
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No depth camera currently connected." << std::endl;
    return;
  }
  
  std::cout << "video_mode [RW]\t\t\t";
  
  breakLines("To set, use 'set video_mode = widthXheight@fps'.", std::cout, 60, "\t\t\t\t");
  
  VideoMode m;
  
  if(_currentDepthCamera->getFrameSize(m.frameSize) && _currentDepthCamera->getFrameRate(m.frameRate))
  {
    std::cout << "Current video mode:" << std::dec << m.frameSize.width << "X" << m.frameSize.height << "@" << m.getFrameRate() << "\n\t\t\t\t";
  }
  
  Vector<SupportedVideoMode> videoModes;
  Set<uint> pixelCounts;
  
  std::cout << "Supported Video Modes: (FPS given below is maximum allowed value) \n\t\t\t\t";
  
  if(_currentDepthCamera->getSupportedVideoModes(videoModes))
  {
    for(auto &v: videoModes)
    {
      std::cout << std::dec << v.frameSize.width << "X" << v.frameSize.height << "@" << v.getFrameRate() << "fps for " << (uint)v.bytesPerPixel*8 << "bpp" 
                << "\n\t\t\t\t";
                
      pixelCounts.insert(v.frameSize.width*v.frameSize.height);
    }
    
    if(pixelCounts.size())
      std::cout << "Supported pixel counts: \n";
    for(auto &p: pixelCounts)
    {
      std::cout << "\t\t\t\t" << p << "\n";
    }
  }
}

void CLIManager::_videoMode(const Vector<String> &tokens)
{
  if(tokens.size() < 2)
  {
    logger(LOG_ERROR) << "Please specify roi parameter" << std::endl;
    return;
  }
  
  // get roi
  if(tokens[0] == "get")
  {
    VideoMode m;
    
    if(!_currentDepthCamera->getFrameSize(m.frameSize) || !_currentDepthCamera->getFrameRate(m.frameRate))
    {
      logger(LOG_ERROR) << "Could not get current video mode" << std::endl;
    }
    else
      std::cout << "video_mode = " << std::dec << m.frameSize.width << "X" << m.frameSize.height << "@" << m.getFrameRate() << "fps" << std::endl;
  }
  else if(tokens[0] == "set")
  {
    if(tokens.size() < 6 || tokens[2] != "=")
    {
      logger(LOG_ERROR) << "Invalid syntax used for setting video_mode" << std::endl;
      _videoModeCapabilities();
      return;
    }
    
    VideoMode m;
    
    char *endptr;
    
    Vector<String> splits;
    
    split(tokens[3], 'X', splits);
    
    if(splits.size() == 1)
      split(tokens[3], 'x', splits);
    
    if(splits.size() != 2)
    {
      logger(LOG_ERROR) << "Please specify frame size as widthXheight" << std::endl;
      _videoModeCapabilities();
      return;
    }
    
    m.frameSize.width = strtol(splits[0].c_str(), &endptr, 10);
    m.frameSize.height = strtol(splits[1].c_str(), &endptr, 10);
    
    float frameRate = strtof(tokens[5].c_str(), &endptr);
    
    m.frameRate.numerator = frameRate*10000;
    m.frameRate.denominator = 10000;
    
    uint g = gcd(m.frameRate.numerator, m.frameRate.denominator);
    m.frameRate.numerator = m.frameRate.numerator/g;
    m.frameRate.denominator = m.frameRate.denominator/g;
    
    std::cout << "Setting video_mode = " << std::dec << m.frameSize.width << "X" << m.frameSize.height << "@" << m.getFrameRate() << "fps ..." << std::endl;
    
    if(!_currentDepthCamera->setFrameSize(m.frameSize) || !_currentDepthCamera->setFrameRate(m.frameRate))
    {
      logger(LOG_ERROR) << "Could not set video_mode" << std::endl;
    }
    else
      std::cout << "Successfully set video_mode" << std::endl;
  }
  else
  {
    logger(LOG_ERROR) << "Unknown command '" << tokens[0] << "' used for video_mode" << std::endl;
  }
}

void CLIManager::_reset(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No depth camera is current connected" << std::endl;
    return;
  }
  
  _stop(tokens);
  
  _viewer->removeDepthCamera();
  _sys.disconnect(_currentDepthCamera, true);
  _currentDepthCamera = nullptr;
}

template <typename T>
void CLIManager::_showFilterSet(const FilterSet<T> &filterSet)
{
  for(auto i = filterSet.begin(); i != filterSet.end(); i++)
  {
    const FilterPtr &p = *i;
    std::cout << i.index << ": " << p->name();
    
    auto &param = p->parameters();
    
    if(param.size())
    {
      for(auto &pr: param)
      {
        std::cout << ", " << pr.first << " = ";
        
        BoolFilterParameter *boolParam = dynamic_cast<BoolFilterParameter *>(pr.second.get());
        EnumFilterParameter *enumParam = dynamic_cast<EnumFilterParameter *>(pr.second.get());
        SignedFilterParameter *signedParam = dynamic_cast<SignedFilterParameter *>(pr.second.get());
        UnsignedFilterParameter *unsignedParam = dynamic_cast<UnsignedFilterParameter *>(pr.second.get());
        FloatFilterParameter *floatParam = dynamic_cast<FloatFilterParameter *>(pr.second.get());
        
        if(boolParam)
        {
          bool v;
          boolParam->get(v);
          std::cout << (v?"true":"false");
          
          const String &m = boolParam->valueMeaning()[v];
          
          if(m.size())
            std::cout << " (" << m << ")";
        }
        
        if(enumParam)
        {
          int v;
          enumParam->get(v);
          
          std::cout << v;
          
          const String &m = boolParam->valueMeaning()[v];
          
          if(m.size())
            std::cout << " (" << m << ")";
        }
        
        if(signedParam)
        {
          int v;
          signedParam->get(v);
          
          std::cout << v;
        }
        
        if(unsignedParam)
        {
          uint v;
          unsignedParam->get(v);
          
          std::cout << v;
        }
        
        if(floatParam)
        {
          float v;
          floatParam->get(v);
          
          std::cout << v;
        }
      }
    }
    std::cout << std::endl;
  }
}


void CLIManager::_filters(const Vector<String> &tokens)
{
  const auto &m = _sys.getSupportedFilters();
  
  if(m.size())
  {
    std::cout << "Supported Filters:" << std::endl;
    
    for(auto &f: m)
      std::cout << f << std::endl;
  }
  
  if(!_currentDepthCamera)
    return;
  
  auto &u = _currentDepthCamera->getUnprocessedRawFilterSet();
  
  if(u.size())
  {
    std::cout << "\nFilters for unprocessed raw frame:" << std::endl;
    
    _showFilterSet(u);
  }
  
  auto &u1 = _currentDepthCamera->getProcessedRawFilterSet();
  
  if(u1.size())
  {
    std::cout << "\nFilters for processed raw frame:" << std::endl;
    
    _showFilterSet(u1);
  }
  
  auto &u2 = _currentDepthCamera->getDepthFilterSet();
  
  if(u2.size())
  {
    std::cout << "\nFilters for depth frame:" << std::endl;
    
    _showFilterSet(u2);
  }
}

void CLIManager::_addFilter2(const String &name, int position, DepthCamera::FrameType type)
{
  FilterPtr p = _sys.createFilter(name, type);
  
  if(!p)
  {
    logger(LOG_ERROR) << "Could not get filter with name '" << name << "'" << std::endl;
    return;
  }
  
  int id = _currentDepthCamera->addFilter(p, type, position);
  
  if(id < 0)
  {
    logger(LOG_ERROR) << "Could not add filter with name '" << name << "'" << std::endl;
    return;
  }
  
  std::cout << "Successfully added filter '" << name << "'. ID = " << id << std::endl;
}


void CLIManager::_addFilter(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No depth camera is current connected" << std::endl;
    return;
  }
  
  if(tokens.size() < 4)
  {
    logger(LOG_ERROR) << "Missing parameters" << std::endl;
    _addFilterHelp();
    return;
  }
  
  if(tokens[1] == "raw")
    _addFilter2(tokens[2], atoi(tokens[3].c_str()), DepthCamera::FRAME_RAW_FRAME_UNPROCESSED);
  else if(tokens[1] == "raw_processed")
    _addFilter2(tokens[2], atoi(tokens[3].c_str()), DepthCamera::FRAME_RAW_FRAME_PROCESSED);
  else if(tokens[1] == "depth")
    _addFilter2(tokens[2], atoi(tokens[3].c_str()), DepthCamera::FRAME_DEPTH_FRAME);
  else
  {
    logger(LOG_ERROR) << "Frame type '" << tokens[1] << "' is not supported" << std::endl;
    return;
  }
}

void CLIManager::_addFilterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(tokens.size() == 1)
  {
    linenoiseAddCompletion(lc, (tokens[0] + " raw").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " raw_processed").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " depth").c_str());   
  }
  else if(tokens.size() == 2)
  {
    if(tokens[1] == "raw" || tokens[1] == "raw_processed" || tokens[1] == "depth")
    {
      const auto &m = _sys.getSupportedFilters();
      
      for(auto &f: m)
        linenoiseAddCompletion(lc, (tokens[0] + " " + tokens[1] + " " + f).c_str());
    }
    else
    {
      Vector<String> f = { "raw", "raw_processed", "depth" };
      
      for(auto &x: f)
        if(x.size() > tokens[1].size() && x.compare(0, tokens[1].size(), tokens[1]) == 0)
          linenoiseAddCompletion(lc, (tokens[0] + " " + x).c_str());
    }
  }
  else if(tokens.size() == 3)
  {
    const auto &m = _sys.getSupportedFilters();
      
    for(auto &f: m)
      if(f.size() > tokens[2].size() && f.compare(0, tokens[2].size(), tokens[2]) == 0)
        linenoiseAddCompletion(lc, (tokens[0] + " " + tokens[1] + " " + f).c_str());
  }
}

void CLIManager::_removeFilter2(int filterID, DepthCamera::FrameType type)
{
  if(!_currentDepthCamera->removeFilter(filterID, type))
  {
    logger(LOG_ERROR) << "Failed to remove filter with ID '" << filterID << "'" << std::endl;
    return;
  }
  
  std::cout << "Successfully removed filter" << std::endl;
}


void CLIManager::_removeFilter(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No depth camera is current connected" << std::endl;
    return;
  }
  
  if(tokens.size() < 3)
  {
    logger(LOG_ERROR) << "Missing parameters" << std::endl;
    _removeFilterHelp();
    return;
  }
  
  if(tokens[1] == "raw")
    _removeFilter2(atoi(tokens[2].c_str()), DepthCamera::FRAME_RAW_FRAME_UNPROCESSED);
  else if(tokens[1] == "raw_processed")
    _removeFilter2(atoi(tokens[2].c_str()), DepthCamera::FRAME_RAW_FRAME_PROCESSED);
  else if(tokens[1] == "depth")
    _removeFilter2(atoi(tokens[2].c_str()), DepthCamera::FRAME_DEPTH_FRAME);
  else
  {
    logger(LOG_ERROR) << "Frame type '" << tokens[1] << "' is not supported" << std::endl;
    return;
  }
}

void CLIManager::_removeFilterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(tokens.size() == 1)
  {
    linenoiseAddCompletion(lc, (tokens[0] + " raw").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " raw_processed").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " depth").c_str());   
  }
  else if(tokens.size() == 2)
  {
    Vector<String> f = { "raw", "raw_processed", "depth" };
      
    for(auto &x: f)
      if(x.size() > tokens[1].size() && x.compare(0, tokens[1].size(), tokens[1]) == 0)
        linenoiseAddCompletion(lc, (tokens[0] + " " + x).c_str());
  }
}

void CLIManager::_setFilterParam2(int filterID, DepthCamera::FrameType type, const String &paramName, const String &paramValue)
{
  FilterPtr p = _currentDepthCamera->getFilter(filterID, type);
  
  if(!p)
  {
    logger(LOG_ERROR) << "No valid filter found for ID '" << filterID << "'" << std::endl;
    return;
  }
  
  auto param = p->getParam(paramName);
  
  if(!param)
  {
    logger(LOG_ERROR) << "No valid parameter '" << paramName << "' found for filter ID '" << filterID << "'" << std::endl;
    return;
  }
  
  const BoolFilterParameter *boolParam = dynamic_cast<const BoolFilterParameter *>(param.get());
  const EnumFilterParameter *enumParam = dynamic_cast<const EnumFilterParameter *>(param.get());
  const SignedFilterParameter *signedParam = dynamic_cast<const SignedFilterParameter *>(param.get());
  const UnsignedFilterParameter *unsignedParam = dynamic_cast<const UnsignedFilterParameter *>(param.get());
  const FloatFilterParameter *floatParam = dynamic_cast<const FloatFilterParameter *>(param.get());
  
  if(boolParam)
  {
    bool v;
    
    if(paramValue == "true" || paramValue == "1")
      v = true;
    else
      v = false;
    
    if(!p->set(paramName, v))
      logger(LOG_ERROR) << "Failed to set parameter '" << paramName << "' for filter with ID '" << filterID << "'" << std::endl;
    else
      std::cout << "Set " << paramName << " = " << paramValue << " successfully for filter with ID '" << filterID << "'" << std::endl;
  }
  
  if(enumParam)
  {
    int v = atoi(paramValue.c_str());
    
    if(!p->set(paramName, v))
      logger(LOG_ERROR) << "Failed to set parameter '" << paramName << "' for filter with ID '" << filterID << "'" << std::endl;
    else
      std::cout << "Set " << paramName << " = " << paramValue << " successfully for filter with ID '" << filterID << "'" << std::endl;
  }
  
  if(signedParam)
  {
    int v = atoi(paramValue.c_str());
    
    if(!p->set(paramName, v))
      logger(LOG_ERROR) << "Failed to set parameter '" << paramName << "' for filter with ID '" << filterID << "'" << std::endl;
    else
      std::cout << "Set " << paramName << " = " << paramValue << " successfully for filter with ID '" << filterID << "'" << std::endl;
  }
  
  if(unsignedParam)
  {
    uint v = atoi(paramValue.c_str());
    
    if(!p->set(paramName, v))
      logger(LOG_ERROR) << "Failed to set parameter '" << paramName << "' for filter with ID '" << filterID << "'" << std::endl;
    else
      std::cout << "Set " << paramName << " = " << paramValue << " successfully for filter with ID '" << filterID << "'" << std::endl;
  }
  
  if(floatParam)
  {
    float v = atof(paramValue.c_str());
    
    if(!p->set(paramName, v))
      logger(LOG_ERROR) << "Failed to set parameter '" << paramName << "' for filter with ID '" << filterID << "'" << std::endl;
    else
      std::cout << "Set " << paramName << " = " << paramValue << " successfully for filter with ID '" << filterID << "'" << std::endl;
  }
}


void CLIManager::_setFilterParam(const Vector<String> &tokens)
{
  if(!_currentDepthCamera)
  {
    logger(LOG_ERROR) << "No depth camera is current connected" << std::endl;
    return;
  }
  
  if(tokens.size() < 6)
  {
    logger(LOG_ERROR) << "Missing parameters" << std::endl;
    _setFilterParamHelp();
    return;
  }
  
  if(tokens[1] == "raw")
    _setFilterParam2(atoi(tokens[2].c_str()), DepthCamera::FRAME_RAW_FRAME_UNPROCESSED, tokens[3], tokens[5]);
  else if(tokens[1] == "raw_processed")
    _setFilterParam2(atoi(tokens[2].c_str()), DepthCamera::FRAME_RAW_FRAME_PROCESSED, tokens[3], tokens[5]);
  else if(tokens[1] == "depth")
    _setFilterParam2(atoi(tokens[2].c_str()), DepthCamera::FRAME_DEPTH_FRAME, tokens[3], tokens[5]);
  else
  {
    logger(LOG_ERROR) << "Frame type '" << tokens[1] << "' is not supported" << std::endl;
    return;
  }
}

void CLIManager::_setFilterParamCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(tokens.size() == 1)
  {
    linenoiseAddCompletion(lc, (tokens[0] + " raw").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " raw_processed").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " depth").c_str());   
  }
  else if(tokens.size() == 2)
  {
    Vector<String> f = { "raw", "raw_processed", "depth" };
    
    for(auto &x: f)
      if(x.size() > tokens[1].size() && x.compare(0, tokens[1].size(), tokens[1]) == 0)
        linenoiseAddCompletion(lc, (tokens[0] + " " + x).c_str());
  }
  else if(tokens.size() == 3)
  {
    if(!_currentDepthCamera)
      return;
    
    FilterPtr p;
    
    int filterID = atoi(tokens[2].c_str());
    
    if(tokens[1] == "raw")
      p = _currentDepthCamera->getFilter(filterID, DepthCamera::FRAME_RAW_FRAME_UNPROCESSED);
    else if(tokens[1] == "raw_processed")
      p = _currentDepthCamera->getFilter(filterID, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    else if(tokens[1] == "depth")
      p = _currentDepthCamera->getFilter(filterID, DepthCamera::FRAME_DEPTH_FRAME);
    
    if(p)
    {
      for(auto &param: p->parameters())
      {
        linenoiseAddCompletion(lc, (tokens[0] + " " + tokens[1] + " " + tokens[2] + " " + param.first).c_str());
      }
    }
  }
  else if(tokens.size() == 4)
  {
    FilterPtr p;
    
    int filterID = atoi(tokens[2].c_str());
    
    if(tokens[1] == "raw")
      p = _currentDepthCamera->getFilter(filterID, DepthCamera::FRAME_RAW_FRAME_UNPROCESSED);
    else if(tokens[1] == "raw_processed")
      p = _currentDepthCamera->getFilter(filterID, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    else if(tokens[1] == "depth")
      p = _currentDepthCamera->getFilter(filterID, DepthCamera::FRAME_DEPTH_FRAME);
    
    if(p)
    {
      for(auto &param: p->parameters())
      {
        if(param.first.size() > tokens[3].size() && param.first.compare(0, tokens[3].size(), tokens[3]) == 0)
          linenoiseAddCompletion(lc, (tokens[0] + " " + tokens[1] + " " + tokens[2] + " " + param.first).c_str());
      }
    }
  }
}

void CLIManager::_vxlToRaw(const Vector<String> &tokens)
{
  if(tokens.size() < 4)
  {
    logger(LOG_ERROR) << "Required parameters missing." << std::endl;
    _vxlToRawHelp();
    return;
  }
  
  String vxlFile = tokens[2];
  _saveFile.close();
  _saveFile.clear();
  _saveFile.open(tokens[3], std::ios::out | std::ios::binary);
    
  if(!_saveFile.good())
  {
    logger(LOG_ERROR) << "Could not open '" << tokens[3] << "' for writing frames." << std::endl;
    _vxlToRawHelp();
    return;
  }
  
  _lastTimeStamp = 0;
  _currentCount = 0;
  
  DepthCamera::FrameType type;
  
  if(tokens[1] == "raw")
    type = DepthCamera::FRAME_RAW_FRAME_UNPROCESSED;
  else if(tokens[1] == "phase" || tokens[1] == "amplitude" || tokens[1] == "ambient" || tokens[1] == "flags")
    type = DepthCamera::FRAME_RAW_FRAME_PROCESSED;
  else if(tokens[1] == "depth")
    type = DepthCamera::FRAME_DEPTH_FRAME;
  else if(tokens[1] == "pointcloud")
    type = DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME;
  else
  {
    logger(LOG_ERROR) << "Unknown frame type '" << tokens[1] << "'" << std::endl;
    _vxlToRawHelp();
    return;
  }
  
  FrameStreamReader r(vxlFile, _sys);
  
  if(!r.isStreamGood())
  {
    logger(LOG_ERROR) << "File could not be opened for reading" << std::endl;
    return;
  }
  
  std::cout << "Number of data frames in stream = " << r.size() << std::endl;
  
  for(auto i = 0; i < r.size(); i++)
  {
    if(!r.readNext())
    {
      logger(LOG_ERROR) << "Could not process frame id = " << i << std::endl;
      continue;
    }
    
    if(type == DepthCamera::FRAME_RAW_FRAME_UNPROCESSED)
      _saveFrameToFile<RawDataFrame>(r.frames[type].get(), _saveFile);
    else if(type == DepthCamera::FRAME_RAW_FRAME_PROCESSED)
      _saveFrameToFile<ToFRawFrame>(r.frames[type].get(), _saveFile, tokens[1]);
    else if(type == DepthCamera::FRAME_DEPTH_FRAME)
      _saveFrameToFile<DepthFrame>(r.frames[type].get(), _saveFile);
    else if(type == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
      _saveFrameToFile<XYZIPointCloudFrame>(r.frames[type].get(), _saveFile);
  }
  
  _saveFile.close();
}

void CLIManager::_vxlToRawCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(!_currentDepthCamera)
    return;
  
  if(tokens.size() == 1)
  {
    linenoiseAddCompletion(lc, (tokens[0] + " raw").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " phase").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " amplitude").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " ambient").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " flags").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " depth").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " pointcloud").c_str());
  }
  else if(tokens.size() == 2)
  {
    Vector<String> types = {"raw", "phase", "amplitude", "ambient", "flags", "depth", "pointcloud"};
    
    for(auto &t: types)
    {
      if(t.size() < tokens[1].size())
        continue;
      
      if(t.compare(0, tokens[1].size(), tokens[1]) == 0)
        linenoiseAddCompletion(lc, (tokens[0] + " " + t).c_str());
    }
  }
}

  
}