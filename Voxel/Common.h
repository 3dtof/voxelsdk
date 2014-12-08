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
#include "Ptr.h"

namespace Voxel
{

template <typename T>
using Vector = std::vector<T>;

template <typename K, typename V>
using Map = std::unordered_map<K, V>;

typedef std::string String;
typedef int IndexType;
typedef std::size_t SizeType;
typedef uint8_t ByteType;

String getHex(uint16_t value);

template<typename T, int sz>
int arraySize(T(&)[sz])
{
  return sz;
}

typedef uint64_t TimeStampType;
TimeStampType getLocalCurrentTime();

}

#endif //VOXEL_COMMON_H