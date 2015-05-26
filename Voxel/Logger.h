/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_LOGGER_H
#define VOXEL_LOGGER_H

#include <iostream>
#include <sstream>
#include <thread>

#include "Common.h"

namespace Voxel
{

/**
 * \addtogroup Util
 * @{
 */
  
enum LogLevel
{
  LOG_CRITICAL,
  LOG_ERROR, 
  LOG_WARNING, 
  LOG_INFO, 
  LOG_DEBUG
};

typedef OutputStream &(*OStreamManipulator)(OutputStream &);

class VOXEL_EXPORT LoggerOutStream
{
public:
  typedef Function<void(const String &)> LoggerOutStreamFunctionType;
protected:
  LoggerOutStreamFunctionType _outputFunction;
  
  OutputStringStream _s;
  
public:
  LoggerOutStream() {}
  
  void setOutputFunction(LoggerOutStreamFunctionType o) { _outputFunction = o; }
  
  template <typename T>
  LoggerOutStream &operator <<(const T &value)
  {
    if(!_outputFunction)
      return *this;
    
    _s << value;
    
    if(_s.tellp() > 0) // Something written?
    {
      _outputFunction(_s.str());
      _s.clear();
      _s.str("");
    }
    return *this;
  }
  
  inline LoggerOutStream &operator <<(OStreamManipulator manip)
  {
    (*manip)(_s);
    return *this;
  }
};
  
class VOXEL_EXPORT Logger
{
protected:
  OutputStream &_out = std::cerr;
  
  mutable Mutex _mutex;

  LogLevel _logLevel, // Allow log statements equal to or below _logLevel
  _currentLogLevel; // This holds log level for current statements
  
  static const String _logLevelNames[5];
  
  Map<IndexType, LoggerOutStream> _outputStreams;
  IndexType _outputStreamCount = 0;
  
public:
  Logger(LogLevel loglevel = LOG_ERROR): _logLevel(loglevel), _currentLogLevel(loglevel)
  {}
  
  Logger &operator =(const Logger &other) { _logLevel = other._logLevel; _currentLogLevel = other._currentLogLevel; return *this; }
  
  inline Logger &operator()(LogLevel loglevel)
  {
    _currentLogLevel = loglevel;
    return *this << _logLevelNames[loglevel] << ": ";
  }
  
  inline LogLevel getDefaultLogLevel()
  {
    return _logLevel;
  }
  
  inline LogLevel getCurrentLogLevel()
  {
    return _currentLogLevel;
  }
  
  inline void setDefaultLogLevel(LogLevel loglevel)
  {
    _logLevel = loglevel;
  }
  
  inline OutputStream &getStream()
  {
    return _out;
  }
  
  inline IndexType addOutputStream(LoggerOutStream::LoggerOutStreamFunctionType f)
  {
    IndexType i = _outputStreamCount;
    _outputStreams[i].setOutputFunction(f);
    _outputStreamCount++;
    return i;
  }
  
  inline bool removeOutputStream(IndexType index)
  {
    auto x = _outputStreams.find(index);
    
    if(x != _outputStreams.end())
    {
      _outputStreams.erase(x);
      return true;
    }
    return false;
  }
  
  template <typename T>
  Logger &operator <<(const T &value)
  {
    if(_currentLogLevel <= _logLevel)
    {
      _out << value;
      
      for(auto &x: _outputStreams)
        x.second << value;
    } 
    return *this;
  }
  
  typedef Logger &(*LoggerManipulator)(Logger &);
  
  inline Logger &operator <<(LoggerManipulator manip)
  {
    if(_currentLogLevel <= _logLevel)
      return (*manip)(*this);
    else
      return *this;
  }
  
  inline Logger &operator <<(OStreamManipulator manip)
  {
    if(_currentLogLevel <= _logLevel)
    {
      (*manip)(_out);
      
      for(auto &x: _outputStreams)
        x.second << manip;
    }
    return *this;
  }
  
  virtual ~Logger()
  {
  }
};

extern Logger VOXEL_EXPORT logger;

//VOXEL_EXPORT Logger & endl(Logger &l);

class VOXEL_EXPORT LogLevelChanger
{
  LogLevel _currentLogLevel;
  LogLevel _desiredLogLevel;
public:
  LogLevelChanger(LogLevel desired): _desiredLogLevel(desired)
  {
    _currentLogLevel = logger.getDefaultLogLevel();
    logger.setDefaultLogLevel(_desiredLogLevel);
  }
  
  ~LogLevelChanger()
  {
    logger.setDefaultLogLevel(_currentLogLevel);
  }
};

/**
 * @}
 */


}
  
#endif // VOXEL_LOGGER_H