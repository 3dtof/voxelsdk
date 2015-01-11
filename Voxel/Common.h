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
#include <unordered_map>
#include <functional>
#include <list>
#include <set>
#include <atomic>

#include <thread>

#include <Ptr.h>

#include <mutex>
#include <condition_variable>

#include "VoxelExports.h"

#define VOXEL_ABI_VERISON 1

namespace Voxel
{

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

typedef std::string String;
typedef int IndexType;
typedef std::size_t SizeType;
typedef uint8_t ByteType;
typedef uint64_t TimeStampType;

#ifdef WINDOWS
typedef uint32_t uint;
#endif

typedef std::thread Thread;
typedef Ptr<Thread> ThreadPtr;

template <typename T>
using Atomic = std::atomic<T>;

typedef std::mutex Mutex;

template <typename T>
using Lock = std::unique_lock<T>;

typedef std::condition_variable ConditionVariable;

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

/// Filesystem functions -- returns the number of files read or else -1 for error. Only those files whose name matches "matchString" partially (sub-string) are returned "files"
int VOXEL_EXPORT getFiles(const String &dir, const String &matchString, Vector<String> &files);

uint VOXEL_EXPORT gcd(uint n, uint m);

#define FLOAT_EPSILON 1E-5f

inline bool floatEquals(float lhs, float rhs)
{
  if((lhs - rhs > FLOAT_EPSILON) || (rhs - lhs > FLOAT_EPSILON))
    return false;
  else
    return true;
}

}

#endif //VOXEL_COMMON_H