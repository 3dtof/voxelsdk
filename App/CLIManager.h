/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_CLIMANAGER_H
#define VOXEL_CLIMANAGER_H

#include <CameraSystem.h>
#include "PCLViewer.h"

namespace Voxel
{
  
struct linenoiseCompletions;

class CLIManager
{
protected:
  CameraSystem &_sys;
  
  DepthCameraPtr _currentDepthCamera;
  
  Ptr<PCLViewer> _viewer;
  
  struct Command
  {
    Function<void(void)> help;
    Function<void(const Vector<String> &tokens)> process;
    Function<void(const Vector<String> &tokens, linenoiseCompletions *lc)> complete;
    
    Command(Function<void(void)> h, Function<void(const Vector<String> &tokens)> p, Function<void(const Vector<String> &tokens, linenoiseCompletions *lc)> c):
    help(h), process(p), complete(c) {}
  };
  
  Map<String, Command> _commands;
  
  bool _keepRunning = true;
  
  void _getTokens(const char *command, Vector<String> &tokens);
  
  void _getPrompt(String &prompt);
  
  void _commandLoop();
  void _completionCallback(const char *buf, linenoiseCompletions *lc);
  
  // Comand related functions
  void _help(const Vector<String> &tokens);
  void _helpHelp();
  
  void _exit(const Vector<String> &tokens);
  void _exitHelp();
  
  void _list(const Vector<String> &tokens);
  void _listHelp();
  
  void _current(const Vector<String> &tokens);
  void _currentHelp();
  
  void _connect(const Vector<String> &tokens);
  void _connectHelp();
  void _connectCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
  void _start(const Vector<String> &tokens);
  void _startHelp();
  
  void _stop(const Vector<String> &tokens);
  void _stopHelp();
  
  void _getRegister(const Vector<String> &tokens);
  void _getRegisterHelp();
  
  void _setRegister(const Vector<String> &tokens);
  void _setRegisterHelp();
  
  void _getParameter(const Vector<String> &tokens);
  void _getParameterHelp();
  void _getParameterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
  void _setParameter(const Vector<String> &tokens);
  void _setParameterHelp();
  void _setParameterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
  void _capabilities(const Vector<String> &tokens);
  void _capabilitiesHelp();
  void _showParameterInfo(const ParameterPtr &param);
  void _capabilitiesCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
  void _paramCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
  void _reset(const Vector<String> &tokens);
  void _resetHelp();
  
  void _disconnect(const Vector<String> &tokens);
  void _disconnectHelp();
  
public:
  CLIManager(CameraSystem &sys);
  
  void run();
  
  virtual ~CLIManager() {}
};

}

#endif // VOXEL_CLIMANAGER_H
