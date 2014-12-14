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

#include <thread>

#include <Ptr.h>

#define DIR_SEP "/"

namespace Voxel
{

template <typename T>
using Vector = std::vector<T>;

template <typename T>
using List = std::list<T>;

template <typename K, typename V>
using Map = std::unordered_map<K, V>;

template <typename T>
using Function = std::function<T>;

typedef std::string String;
typedef int IndexType;
typedef std::size_t SizeType;
typedef uint8_t ByteType;
typedef uint64_t TimeStampType;

typedef std::thread Thread;
typedef Ptr<Thread> ThreadPtr;


/// String functions
String getHex(uint16_t value);
void split(const String &str, const char delimiter, Vector<String> &split);

/// Array handling template
template<typename T, int sz>
int arraySize(T(&)[sz])
{
  return sz;
}

/// Filesystem functions -- returns the number of files read or else -1 for error. Only those files whose name matches "matchString" partially (sub-string) are returned "files"
int getFiles(const String &dir, const String &matchString, Vector<String> &files);

uint gcd(uint n, uint m);

}

#endif //VOXEL_COMMON_H