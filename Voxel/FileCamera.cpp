/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "FileCamera.h"

namespace Voxel
{
// Member functions definitions including constructor
// FileCamera::FileCamera( double len)
// {
    // length = len;
// }

void FileCamera::setLength( double len )
{
    _length = len;
}

FileCamera::~FileCamera()
{
}

}