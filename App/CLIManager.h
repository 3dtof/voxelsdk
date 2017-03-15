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
    tFunction<void(void)> help;
    tFunction<void(const tVector<String> &tokens)> process;
    tFunction<void(const tVector<String> &tokens, linenoiseCompletions *lc)> complete;
    
    Command(tFunction<void(void)> h, tFunction<void(const tVector<String> &tokens)> p, tFunction<void(const tVector<String> &tokens, linenoiseCompletions *lc)> c):
    help(h), process(p), complete(c) {}
  };
  
  struct CommandGroup
  {
    String name;
    tVector<String> commands;
  };
  
  tMap<String, Command> _commands;
  tVector<CommandGroup> _commandGroups; // Holds groups of commands, suitable for printing help
  tMap<String, Command> _specialParameters; // such as frame_size, frame_rate, roi, etc.
  
  String _commandHistoryFileName;
  
  bool _keepRunning;
  
  void _getTokens(const char *command, tVector<String> &tokens);
  
  void _getPrompt(String &prompt);
  
  void _commandLoop();
  void _completionCallback(const char *buf, LineNoise::linenoiseCompletions *lc);
  
  // Comand related functions
  void _help(const tVector<String> &tokens);
  void _helpHelp();
  void _helpCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  void _exit(const tVector<String> &tokens);
  void _exitHelp();
  
  void _list(const tVector<String> &tokens);
  void _listHelp();
  
  void _current(const tVector<String> &tokens);
  void _currentHelp();
  
  void _connect(const tVector<String> &tokens);
  void _connectHelp();
  void _connectCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  void _start(const tVector<String> &tokens);
  void _startHelp();
  
  void _stop(const tVector<String> &tokens);
  void _stopHelp();
  
  void _getRegister(const tVector<String> &tokens);
  void _getRegisterHelp();
  
  void _setRegister(const tVector<String> &tokens);
  void _setRegisterHelp();
  
  void _getParameter(const tVector<String> &tokens);
  void _getParameterHelp();
  void _getParameterCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  void _setParameter(const tVector<String> &tokens);
  void _setParameterHelp();
  void _setParameterCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  void _capabilities(const tVector<String> &tokens);
  void _capabilitiesHelp();
  void _showParameterInfo(const ParameterPtr &param);
  void _capabilitiesCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  void _paramCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  // All special parameter related functions
  void _roiCapabilities(); // Use to show capabilities
  void _roi(const tVector<String> &tokens); // Both for get and set
  void _videoModeCapabilities(); // Use to show capabilities
  void _videoMode(const tVector<String> &tokens); // Both for get and set
  
  TimeStampType _lastTimeStamp;
  std::ofstream _saveFile;
  String _saveFileName;
  int _count, _currentCount;
  
  boost::signals2::connection _saveCallbackConnection;
  
  void _save(const tVector<String> &tokens);
  void _saveHelp();
  
  template <typename FrameType>
  bool _saveFrameToFile(const Frame *frame, OutputFileStream &saveFile, const String &subType = "");
  
  void _saveCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  void _vxlToRaw(const tVector<String> &tokens);
  void _vxlToRawHelp();
  void _vxlToRawCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  
  void _reset(const tVector<String> &tokens);
  void _resetHelp();
  
  void _disconnect(const tVector<String> &tokens);
  void _disconnectHelp();
  
  
  template <typename T>
  void _showFilterSet(const FilterSet<T> &filterSet);
  
  void _filters(const tVector<String> &tokens);
  void _filtersHelp();
  
  void _addFilter2(const String &name, int position, DepthCamera::FrameType type);
  void _addFilter(const tVector<String> &tokens);
  void _addFilterCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _addFilterHelp();
  
  void _removeFilter2(int filterID, DepthCamera::FrameType type);
  void _removeFilter(const tVector<String> &tokens);
  void _removeFilterCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _removeFilterHelp();
  
  void _setFilterParam2(int filterID, DepthCamera::FrameType type, const String &paramName, const String &paramValue);
  void _setFilterParam(const tVector<String> &tokens);
  void _setFilterParamCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _setFilterParamHelp();
  
  void _profileList(const tVector<String> &tokens);
  void _profileListHelp();
  
  void _profileSet(const tVector<String> &tokens);
  
  /// @param type is a 2-bit field. LSB is to include software profiles and MSB for hardware profile. By default, both are included.
  void _getProfileIDs(const String &partialID, tVector<String> &ids, const unsigned int type = 0x3); 
  void _profileSetCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileSetHelp();
  
  void _profileAdd(const tVector<String> &tokens);
  void _profileAddCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileAddHelp();
  
  void _profileRemove(const tVector<String> &tokens);
  void _profileRemoveCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileRemoveHelp();
  
  void _profileParam(const tVector<String> &tokens);
  void _profileParamCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileParamHelp();
  
  void _profileParamRemove(const tVector<String> &tokens);
  void _profileParamRemoveCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileParamRemoveHelp();
  
  void _profileSetDefault(const tVector<String> &tokens);
  void _profileSetDefaultCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileSetDefaultHelp();
  
  void _profileHWFetch(const tVector<String> &tokens);
  void _profileHWFetchCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileHWFetchHelp();
  
  void _profileHWSave(const tVector<String> &tokens);
  void _profileHWSaveCompletion(const tVector<String> &tokens, linenoiseCompletions *lc);
  void _profileHWSaveHelp();
  
  void _profileHWRemoveDefault(const tVector<String> &tokens);
  void _profileHWRemoveDefaultHelp();
  
public:
  CLIManager(CameraSystem &sys);
  
  void run();
  
  virtual ~CLIManager() {}
};

}

#endif // VOXEL_CLIMANAGER_H
