/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_COMMON_H
#define VOXEL_COMMON_H

#include <vector>
#include <stdint.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <functional>
#include <list>
#include <set>
#include <atomic>
#include <cctype>

#include <algorithm>

#include <complex>

#include <thread>

#include <Ptr.h>

#include <mutex>
#include <condition_variable>

#include "VoxelConfig.h"
#include "VoxelExports.h"

#ifdef _WIN32
typedef uint32_t uint;
#endif

namespace Voxel
{
  
/**
 * \defgroup Util Common and Utility classes
 * @{
 */

#ifndef SWIG
template <typename T>
using Vector = std::vector<T>;

template <typename T>
using List = std::list<T>;

template <typename T>
using Set = std::set<T>;

template <typename K, typename V>
using Map = std::unordered_map<K, V>;

template <typename T>
using Function = std::function<T>;

template <typename T>
using Atomic = std::atomic<T>;

template <typename T>
using Lock = std::unique_lock<T>;

template <typename T>
using shared_ptr = Ptr<T>;
#else
#define Vector std::vector
#define List std::list
#define Set std::set
#define Map std::unordered_map
#define Function std::function
#define Atomic std::atomic
#define Lock std::unique_lock
#endif

typedef std::string String;
typedef int IndexType;
typedef std::size_t SizeType;
typedef uint8_t ByteType;
typedef uint64_t TimeStampType;
typedef uint32_t GeneratorIDType;
typedef uint64_t FileOffsetType;


typedef std::complex<float> Complex;
typedef std::complex<double> ComplexDouble;

typedef std::thread Thread;
typedef Ptr<Thread> ThreadPtr;
typedef std::mutex Mutex;
typedef std::condition_variable ConditionVariable;

typedef std::ifstream InputFileStream;
typedef std::ofstream OutputFileStream;

typedef std::istream InputStream;
typedef std::ostream OutputStream;
typedef std::ostringstream OutputStringStream;
typedef std::istringstream InputStringStream;

struct Version { uint8_t major, minor; };

/// String functions
String VOXEL_EXPORT getHex(uint16_t value);
void VOXEL_EXPORT split(const String &str, const char delimiter, Vector<String> &split);
void VOXEL_EXPORT breakLines(const String &str, std::ostream &out, const uint maxPerLine, const String &newlinePrefix);

/// Array handling template
template<typename T, int sz>
int arraySize(T(&)[sz])
{
  return sz;
}

// Allocate in bytes and irrespective of size required by 'T'. This is useful if 'T' is has data structures which could have extendable elements. 
// E.g.: Windows USB device handling structures in usbioctl.h
template <typename T>
Ptr<T> byteAlloc(unsigned long sizeInBytes)
{
  return Ptr<T>((T *)new uint8_t[sizeInBytes], [](T *d) { delete[]((uint8_t *)d);  });
}

float VOXEL_EXPORT gcd(float n, float m);

// This returns nearest 'n = 2^m' such that 2^(m - 1) < value < 2^m
unsigned int VOXEL_EXPORT nearestPowerOf2(unsigned int value, unsigned int &index);

#define FLOAT_EPSILON 1E-5f

inline bool floatEquals(float lhs, float rhs)
{
  if((lhs - rhs > FLOAT_EPSILON) || (rhs - lhs > FLOAT_EPSILON))
    return false;
  else
    return true;
}

/**
 * @brief Left Trim
 *
 * Trims whitespace from the left end of the provided String
 *
 * @param[out] s The String to trim
 *
 * @return The modified String&
 */
inline String& ltrim(String& s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                  std::ptr_fun<int, int>(std::isgraph)));
  return s;
}

/**
 * @brief Right Trim
 *
 * Trims whitespace from the right end of the provided String
 *
 * @param[out] s The String to trim
 *
 * @return The modified String&
 */
inline String& rtrim(String& s) {
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       std::ptr_fun<int, int>(std::isgraph)).base(), s.end());
  return s;
}

/**
 * @brief Trim
 *
 * Trims whitespace from both ends of the provided String
 *
 * @param[out] s The String to trim
 *
 * @return The modified String&
 */
inline String& trim(String& s) {
  return ltrim(rtrim(s));
}

/// Filesystem functions -- returns the number of files read or else -1 for error. Only those files whose name matches "matchString" partially (sub-string) are returned "files"
int VOXEL_EXPORT getFiles(const String &dir, const String &matchString, Vector<String> &files);

bool VOXEL_EXPORT isFilePresent(const String &filename);
bool VOXEL_EXPORT isDirectory(const String &filename);
bool VOXEL_EXPORT makeDirectory(const String &filename);
String VOXEL_EXPORT dirname(const String &filename);
String VOXEL_EXPORT basename(const String &filename);

String VOXEL_EXPORT substitute(const String &s, const Vector<String> &keys, const Vector<String> &values);

/**
 * @}
 */

}

#endif //VOXEL_COMMON_H
