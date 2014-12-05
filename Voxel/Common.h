/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_COMMON_H
#define VOXEL_COMMON_H

#include <vector>
#include <memory>
#include <stdint.h>
#include <string>
#include <iostream>
#include <unordered_map>

namespace Voxel
{

template <typename T>
using Vector = std::vector<T>;

template <typename K, typename V>
using Map = std::unordered_map<K, V>;

template <typename T>
using Ptr = std::shared_ptr<T>;

typedef std::string String;
typedef int IndexType;
typedef std::size_t SizeType;
typedef uint8_t ByteType;

String getHex(uint16_t value);
}

#endif //VOXEL_COMMON_H