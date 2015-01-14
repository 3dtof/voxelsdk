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

namespace Voxel
{
  
CLIManager::CLIManager(CameraSystem &sys): _sys(sys)
{
  _commands = Map<String, Command>({
    {"list",      Command(_H(&CLIManager::_listHelp),         _P(&CLIManager::_list),         nullptr)},
    {"status",    Command(_H(&CLIManager::_currentHelp),      _P(&CLIManager::_current),      nullptr)},
    {"connect",   Command(_H(&CLIManager::_connectHelp),      _P(&CLIManager::_connect),      _C(&CLIManager::_connectCompletion))},
    {"start",     Command(_H(&CLIManager::_startHelp),        _P(&CLIManager::_start),        nullptr)},
    {"stop",      Command(_H(&CLIManager::_stopHelp),         _P(&CLIManager::_stop),         nullptr)},
    {"getr",      Command(_H(&CLIManager::_getRegisterHelp),  _P(&CLIManager::_getRegister),  nullptr)},
    {"setr",      Command(_H(&CLIManager::_setRegisterHelp),  _P(&CLIManager::_setRegister),  nullptr)},
    {"get",       Command(_H(&CLIManager::_getParameterHelp), _P(&CLIManager::_getParameter), _C(&CLIManager::_getParameterCompletion))},
    {"set",       Command(_H(&CLIManager::_setParameterHelp), _P(&CLIManager::_setParameter), _C(&CLIManager::_setParameterCompletion))},
    {"save",      Command(_H(&CLIManager::_saveHelp),         _P(&CLIManager::_save),         _C(&CLIManager::_saveCompletion))},
    {"cap",       Command(_H(&CLIManager::_capabilitiesHelp), _P(&CLIManager::_capabilities), _C(&CLIManager::_capabilitiesCompletion))},
    {"help",      Command(_H(&CLIManager::_helpHelp),         _P(&CLIManager::_help),         nullptr)},
    {"disconnect",Command(_H(&CLIManager::_disconnectHelp),   _P(&CLIManager::_disconnect),   nullptr)},
    {"reset",     Command(_H(&CLIManager::_resetHelp),        _P(&CLIManager::_reset),        nullptr)},
    {"exit",      Command(_H(&CLIManager::_exitHelp),         _P(&CLIManager::_exit),         nullptr)},
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
    else if(isalnum(*c) || *c == '_' || *c == '.' || *c == '-')
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


void CLIManager::_helpHelp()          { std::cout << "help [cmd]\t\t  Print this help and for any specified Voxel command 'cmd'" << std::endl; }
void CLIManager::_exitHelp()          { std::cout << "exit\t\t\t  Exit VoxelCLI" << std::endl; }
void CLIManager::_connectHelp()       { std::cout << "connect <dev>\t\t  Connect to a specific device. <dev> needs to be device ID\n"
                                                  << "\t\t\t  in the format INTERFACE::DEVICE::SERIAL. Use double-quotes for the ID if it contains spaces" << std::endl; }
void CLIManager::_listHelp()          { std::cout << "list\t\t\t  Scan and list all valid Voxel connectable devices" << std::endl; }
void CLIManager::_currentHelp()       { std::cout << "status\t\t\t  Show the current connected device ID if present" << std::endl; }
void CLIManager::_startHelp()         { std::cout << "start\t\t\t  Start streaming and showing the current device" << std::endl; }
void CLIManager::_stopHelp()          { std::cout << "stop\t\t\t  Stop streaming the current device" << std::endl; }
void CLIManager::_getRegisterHelp()   { std::cout << "getr <reg>\t\t  Get register value at <reg>. Use '0x' prefix for hexadecimal" << std::endl; }
void CLIManager::_setRegisterHelp()   { std::cout << "setr <reg> = <value>\t  Set register <reg> to <value>. Use '0x' prefix for hexadecimal" << std::endl; }
void CLIManager::_getParameterHelp()  { std::cout << "get <param>\t\t  Get parameter value given by name <param>" << std::endl; }
void CLIManager::_capabilitiesHelp()  { std::cout << "cap [<param>][*]\t  Get capabilities of the current depth camera.\n"
                                                  << "\t\t\t  Optionally a parameter name can be given to list only that parameter details given by name <param>.\n"
                                                  << "\t\t\t  A optional wildcard can be given to list all parameters beginning with name <param>" << std::endl; }
void CLIManager::_setParameterHelp()  { std::cout << "set <param> = <value>\t  Set parameter value given by name <param>. Use '0x' prefix for hexadecimal" << std::endl; }
void CLIManager::_saveHelp()          { std::cout << "save type count filename  Save current 'count' number of frames.\n"
                                                  <<  "\t\t\t  'type' = raw/raw_processed/depth/pointcloud" << std::endl; }
void CLIManager::_disconnectHelp()    { std::cout << "disconnect\t\t  Disconnect the currently connected depth camera" << std::endl; }
void CLIManager::_resetHelp()         { std::cout << "reset\t\t\t  Reset and disconnect the currently connected depth camera" << std::endl; }


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

void CLIManager::_current(const Vector< String > &tokens)
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
  
  const BoolParameter *boolParam = dynamic_cast<const BoolParameter *>(param.get());
  const IntegerParameter *intParam = dynamic_cast<const IntegerParameter *>(param.get());
  const UnsignedIntegerParameter *uintParam = dynamic_cast<const UnsignedIntegerParameter *>(param.get());
  const FloatParameter *floatParam = dynamic_cast<const FloatParameter *>(param.get());
  const EnumParameter *enumParam = dynamic_cast<const EnumParameter *>(param.get());
  
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
    return;
  }
  
  _currentCount = 0;
  _lastTimeStamp = 0;
  
  _saveFile.open(tokens[3], std::ios::out | std::ios::binary);
  
  if(!_saveFile.good())
  {
    logger(LOG_ERROR) << "Could not open '" << tokens[3] << "' for writing frames." << std::endl;
    return;
  }
  
  _saveFileName = tokens[3];
  
  if(tokens[1] == "raw")
  {
    _saveCallbackConnection = _viewer->getGrabber()->registerCallback<PCLGrabber::RawImageCallBack>([this](const Voxel::RawFrame &frame, const Voxel::DepthCamera::FrameCallBackType type) -> void {
      if(type != DepthCamera::CALLBACK_RAW_FRAME_UNPROCESSED)
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
      
      _saveFile.write((char *)&d->id, sizeof(d->id));
      _saveFile.write((char *)&d->timestamp, sizeof(d->timestamp));
      
      _saveFile.write((char *)d->data.data(), d->data.size());
      
      _currentCount++;
      
      if(_currentCount >= _count)
      {
        std::cout << "Saved to file '" << _saveFileName << "'" << std::endl;
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
    });
  } 
  else if(tokens[1] == "raw_processed")
  {
    _saveCallbackConnection = _viewer->getGrabber()->registerCallback<PCLGrabber::RawImageCallBack>([this](const Voxel::RawFrame &frame, const Voxel::DepthCamera::FrameCallBackType type) {
      if(type != DepthCamera::CALLBACK_RAW_FRAME_PROCESSED)
        return;
      
      const ToFRawFrame *d = dynamic_cast<const ToFRawFrame *>(&frame);

      if(!d)
      {
        logger(LOG_ERROR) << "Null frame captured? or not of type ToFRawFrame" << std::endl;
        return;
      }

      std::cout << "Capture frame " << d->id << "@" << d->timestamp;

      if(_lastTimeStamp != 0)
      {
        std::cout << " (" << 1E6 / (d->timestamp - _lastTimeStamp) << " fps)";
      }

      std::cout << std::endl;

      _lastTimeStamp = d->timestamp;
      
      _saveFile.write((char *)&d->id, sizeof(d->id));
      _saveFile.write((char *)&d->timestamp, sizeof(d->timestamp));

      if(d->phase())
      {
        _saveFile.write((char *)d->phase(), d->phaseWordWidth()*d->size.width * d->size.height);
      }

      if(d->amplitude())
      {
        _saveFile.write((char *)d->amplitude(), d->amplitudeWordWidth()*d->size.width * d->size.height);
      }

      if(d->ambient())
      {
        _saveFile.write((char *)d->ambient(), d->ambientWordWidth()*d->size.width * d->size.height);
      }

      if(d->flags())
      {
        _saveFile.write((char *)d->flags(), d->flagsWordWidth()*d->size.width * d->size.height);      
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
      const DepthFrame *d = dynamic_cast<const DepthFrame *>(&frame);
      
      if(!d)
      {
        logger(LOG_ERROR) << "Null frame captured? or not of type DepthFrame" << std::endl;
        return;
      }
      
      std::cout << "Capture frame " << d->id << "@" << d->timestamp;
      
      if(_lastTimeStamp != 0)
        std::cout << " (" << 1E6/(d->timestamp - _lastTimeStamp) << " fps)";
      
      std::cout << std::endl;
      
      _lastTimeStamp = d->timestamp;
      
      _saveFile.write((char *)&d->id, sizeof(d->id));
      _saveFile.write((char *)&d->timestamp, sizeof(d->timestamp));
      
      _saveFile.write((char *)d->depth.data(), sizeof(float)*d->size.width*d->size.height);
      _saveFile.write((char *)d->amplitude.data(), sizeof(float)*d->size.width*d->size.height);
    
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
      const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
      
      if(!d)
      {
        logger(LOG_ERROR) << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
        return;
      }
      
      std::cout << "Capture frame " << d->id << "@" << d->timestamp;
      
      if(_lastTimeStamp != 0)
        std::cout << " (" << 1E6/(d->timestamp - _lastTimeStamp) << " fps)";
      
      std::cout << std::endl;
      
      _lastTimeStamp = d->timestamp;
      
      _saveFile.write((char *)&d->id, sizeof(d->id));
      _saveFile.write((char *)&d->timestamp, sizeof(d->timestamp));
      
      _saveFile.write((char *)d->points.data(), sizeof(IntensityPoint)*d->points.size());
          
      _currentCount++;
      
      if(_currentCount >= _count)
      {
        std::cout << "Saved to file '" << _saveFileName << "'" << std::endl;
        _saveFile.close();
        _saveCallbackConnection.disconnect();
      }
    });
  }
  else
  {
    logger(LOG_ERROR) << "Unknown frame type '" << tokens[1] << "'" << std::endl;
    return;
  }
  
  if(!_saveCallbackConnection.connected())
  {
    logger(LOG_ERROR) << "Could not connect to save the frames." << std::endl;
    _saveFile.close();
  }
}

void CLIManager::_saveCompletion(const Vector<String> &tokens, linenoiseCompletions *lc)
{
  if(!_currentDepthCamera)
    return;
  
  if(tokens.size() == 1)
  {
    linenoiseAddCompletion(lc, (tokens[0] + " raw").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " raw_processed").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " depth").c_str());
    linenoiseAddCompletion(lc, (tokens[0] + " pointcloud").c_str());
  }
  else if(tokens.size() == 2)
  {
    Vector<String> types = {"raw", "raw_processed", "depth", "pointcloud"};
    
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

void CLIManager::_reset(const Vector< String > &tokens)
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



  
}