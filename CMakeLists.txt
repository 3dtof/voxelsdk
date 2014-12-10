cmake_minimum_required (VERSION 2.6)
project (TIVoxel)

MESSAGE(${CMAKE_SYSTEM_NAME})

add_definitions(-g -msse2 -pthread -fopenmp -std=c++11)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux") # in the binary package DEB, add dependency for libusb-1.0
  set(COMMON_LIBS ${COMMON_LIBS} usb-1.0 dl)
  set(COMMON_INCLUDE
    ${COMMON_INCLUDE}
    /usr/include/libusb-1.0
  )
endif()

include_directories(
  Voxel
  TI3DToF
  ${COMMON_INCLUDE}
)

add_subdirectory(Voxel)
add_subdirectory(Test)
add_subdirectory(TI3DToF)
