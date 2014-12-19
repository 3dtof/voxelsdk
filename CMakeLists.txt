cmake_minimum_required (VERSION 2.8.11)
project (TIVoxel)

#set(CMAKE_CXX_VISIBILITY_PRESET hidden)
#set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

MESSAGE(${CMAKE_SYSTEM_NAME})

add_definitions(-msse2 -pthread -fopenmp -std=c++11)

set(ARCH ${CMAKE_SYSTEM_PROCESSOR})

if(CMAKE_SYSTEM_NAME STREQUAL "Linux") # in the binary package DEB, add dependency for libusb-1.0
  set(COMMON_LIBS ${COMMON_LIBS} usb-1.0 dl)
  set(COMMON_INCLUDE
    ${COMMON_INCLUDE}
    /usr/include/libusb-1.0
  )
  
  if(ARCH STREQUAL "x86_64")
    set(ARCH amd64)
  endif()
  
  file(WRITE "${CMAKE_BINARY_DIR}/make_deb.sh" "for i in CPackConfig-*.cmake; do cpack -G DEB --config $i; done\n")
  
endif()

include_directories(
  Voxel
  TI3DToF
  ${COMMON_INCLUDE}
)

#### Packaging related information
function(create_cpack_config name version)
  set(CPACK_PACKAGE_VERSION "${version}")
  set(CPACK_PACKAGE_NAME "${name}")
  set(CPACK_PACKAGE_FILE_NAME "${name}-${version}-${ARCH}")
  set(CPACK_OUTPUT_CONFIG_FILE "${CMAKE_BINARY_DIR}/CPackConfig-${name}.cmake")
  include(CPack)
endfunction(create_cpack_config)

set(CPACK_PACKAGE_NAME ti-voxel)
set(CPACK_PACKAGE_VERSION 0.1)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${ARCH})
set(CPACK_PACKAGE_CONTACT "prasad.bhat@ti.com")
set(CPACK_PACKAGE_VENDOR "Texas Instruments, Inc.")
set(CPACK_DEBIAN_PACKAGE_SECTION "Electronics")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library package. Contains the main Voxel library.")
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)
set(CPACK_DEB_COMPONENT_INSTALL 1)

add_subdirectory(Voxel)
add_subdirectory(Test)
add_subdirectory(TI3DToF)
add_subdirectory(App)

#include(CPack)