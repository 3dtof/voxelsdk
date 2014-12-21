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
  
enum LogLevel
{
  LOG_CRITICAL,
  LOG_ERROR, 
  LOG_WARNING, 
  LOG_INFO, 
  LOG_DEBUG
};
  
class Logger
{
protected:
  std::ostream &_out = std::cerr;

  LogLevel _logLevel, // Allow log statements equal to or below _logLevel
  _currentLogLevel; // This holds log level for current statements
  
  static const String _logLevelNames[5];
  
public:
  Logger(LogLevel loglevel = LOG_ERROR): _logLevel(loglevel), _currentLogLevel(loglevel)
  {}
  
  inline Logger &operator()(LogLevel loglevel)
  {
    _currentLogLevel = loglevel;
    return *this << _logLevelNames[loglevel] << ": ";
  }
  
  inline LogLevel getDefaultLogLevel()
  {
    return _logLevel;
  }
  
  inline void setDefaultLogLevel(LogLevel loglevel)
  {
    _logLevel = loglevel;
  }
  
  inline std::ostream &getStream()
  {
    return _out;
  }
  
  template <typename T>
  Logger &operator <<(const T &value)
  {
    if(_currentLogLevel <= _logLevel)
      _out << value;
    return *this;
  }
  
  typedef Logger &(*LoggerManipulator)(Logger &);
  
  typedef std::ostream &(*OStreamManipulator)(std::ostream &);
  
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
      (*manip)(_out);
    return *this;
  }
  
  virtual ~Logger()
  {
  }
};

extern Logger logger;

Logger &endl(Logger &l);

class LogLevelChanger
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

}
  
#endif // VOXEL_LOGGER_H