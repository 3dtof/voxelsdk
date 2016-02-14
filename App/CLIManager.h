/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_CLIMANAGER_H
#define VOXEL_CLIMANAGER_H

#include <CameraSystem.h>
#include "PCLViewer.h"
#include <fstream>

#include <boost/signals2/connection.hpp>

namespace LineNoise
{  
struct linenoiseCompletions;
}

using namespace LineNoise;

namespace Voxel
{

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
  
  struct CommandGroup
  {
    String name;
    Vector<String> commands;
  };
  
  Map<String, Command> _commands;
  Vector<CommandGroup> _commandGroups; // Holds groups of commands, suitable for printing help
  Map<String, Command> _specialParameters; // such as frame_size, frame_rate, roi, etc.
  
  String _commandHistoryFileName;
  
  bool _keepRunning = true;
  
  void _getTokens(const char *command, Vector<String> &tokens);
  
  void _getPrompt(String &prompt);
  
  void _commandLoop();
  void _completionCallback(const char *buf, LineNoise::linenoiseCompletions *lc);
  
  // Comand related functions
  void _help(const Vector<String> &tokens);
  void _helpHelp();
  void _helpCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
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
  
  // All special parameter related functions
  void _roiCapabilities(); // Use to show capabilities
  void _roi(const Vector<String> &tokens); // Both for get and set
  void _videoModeCapabilities(); // Use to show capabilities
  void _videoMode(const Vector<String> &tokens); // Both for get and set
  
  TimeStampType _lastTimeStamp;
  std::ofstream _saveFile;
  String _saveFileName;
  int _count, _currentCount;
  
  boost::signals2::connection _saveCallbackConnection;
  
  void _save(const Vector<String> &tokens);
  void _saveHelp();
  
  template <typename FrameType>
  bool _saveFrameToFile(const Frame *frame, OutputFileStream &saveFile, const String &subType = "");
  
  void _saveCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
  void _vxlToRaw(const Vector<String> &tokens);
  void _vxlToRawHelp();
  void _vxlToRawCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  
  void _reset(const Vector<String> &tokens);
  void _resetHelp();
  
  void _disconnect(const Vector<String> &tokens);
  void _disconnectHelp();
  
  
  template <typename T>
  void _showFilterSet(const FilterSet<T> &filterSet);
  
  void _filters(const Vector<String> &tokens);
  void _filtersHelp();
  
  void _addFilter2(const String &name, int position, DepthCamera::FrameType type);
  void _addFilter(const Vector<String> &tokens);
  void _addFilterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  void _addFilterHelp();
  
  void _removeFilter2(int filterID, DepthCamera::FrameType type);
  void _removeFilter(const Vector<String> &tokens);
  void _removeFilterCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  void _removeFilterHelp();
  
  void _setFilterParam2(int filterID, DepthCamera::FrameType type, const String &paramName, const String &paramValue);
  void _setFilterParam(const Vector<String> &tokens);
  void _setFilterParamCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  void _setFilterParamHelp();
  
  void _profileList(const Vector<String> &tokens);
  void _profileListHelp();
  
  void _profileSet(const Vector<String> &tokens);
  void _getProfileIDs(const String &partialID, Vector<String> &ids);
  void _profileSetCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  void _profileSetHelp();
  
  void _profileAdd(const Vector<String> &tokens);
  void _profileAddCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  void _profileAddHelp();
  
  void _profileRemove(const Vector<String> &tokens);
  void _profileRemoveCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  void _profileRemoveHelp();
  
  void _profileParam(const Vector<String> &tokens);
  void _profileParamCompletion(const Vector<String> &tokens, linenoiseCompletions *lc);
  void _profileParamHelp();
  
public:
  CLIManager(CameraSystem &sys);
  
  void run();
  
  virtual ~CLIManager() {}
};

}

#endif // VOXEL_CLIMANAGER_H
