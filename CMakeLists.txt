cmake_minimum_required (VERSION 2.8.11)
project (TIVoxel)

set(VOXEL_MAJOR_VERSION 0)
set(VOXEL_MINOR_VERSION 1)
set(VOXEL_PATCH_VERSION 0)
set(VOXEL_VERSION
  ${VOXEL_MAJOR_VERSION}.${VOXEL_MINOR_VERSION}.${VOXEL_PATCH_VERSION})

### Do not export any symbol by default
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
include(GenerateExportHeader)


set(CMAKE_INCLUDE_CURRENT_DIR 1)

MESSAGE(${CMAKE_SYSTEM_NAME})

set(ARCH ${CMAKE_SYSTEM_PROCESSOR})

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  SET(LINUX 1)
  add_definitions(-msse2 -pthread -fopenmp -std=c++11)
  ADD_DEFINITIONS(-DLINUX)  

  set(COMMON_LIBS ${COMMON_LIBS} usb-1.0 dl)
  SET(COMMON_LIBS_PRIVATE "")
  set(COMMON_INCLUDE
    ${COMMON_INCLUDE}
    /usr/include/libusb-1.0
  )
  
  if(ARCH STREQUAL "x86_64")
    set(ARCH amd64)
  endif()
  
  file(WRITE "${CMAKE_BINARY_DIR}/make_deb.sh" "for i in CPackConfig-*.cmake; do cpack -G DEB --config $i; done\n")
  
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  SET(WINDOWS 1)
  ADD_DEFINITIONS(-DWINDOWS /wd4251)
  SET(COMMON_LIBS setupapi WinUsb strmiids strmbase Cfgmgr32)
  SET(COMMON_LIBS_PRIVATE ${CMAKE_SOURCE_DIR}/Voxel/CyAPI.lib)
endif()

include_directories(
  Voxel
  TI3DToF
  ${CMAKE_BINARY_DIR}/Voxel
  ${CMAKE_BINARY_DIR}/TI3DToF
  ${COMMON_INCLUDE}
)

IF(LINUX)
  function(create_cpack_config name version)
    set(CPACK_PACKAGE_VERSION "${version}")
    set(CPACK_PACKAGE_NAME "${name}")
    set(CPACK_PACKAGE_FILE_NAME "${name}-${version}-${ARCH}")
    set(CPACK_OUTPUT_CONFIG_FILE "${CMAKE_BINARY_DIR}/CPackConfig-${name}.cmake")
    include(CPack)
  endfunction(create_cpack_config)
  
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${ARCH})
  set(CPACK_DEBIAN_PACKAGE_SECTION "Electronics")
  set(CPACK_DEB_COMPONENT_INSTALL 1)
ENDIF()

INCLUDE(common.cmake)
  
#### Packaging related information -- common to both platforms
set(CPACK_PACKAGE_NAME "Voxel SDK")
set(CPACK_PACKAGE_VERSION ${VOXEL_VERSION})
set(CPACK_PACKAGE_CONTACT "prasad.bhat@ti.com")
set(CPACK_PACKAGE_VENDOR "Texas Instruments, Inc.")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library package. Contains the main Voxel library.")
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)

add_subdirectory(Voxel)
add_subdirectory(Test)
add_subdirectory(TI3DToF)
add_subdirectory(App)

IF(WINDOWS)
  SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
  SET(CPACK_NSIS_MODIFY_PATH ON)
  SET(CPACK_PACKAGE_EXECUTABLES 
    "VoxelCLI;Voxel CLI"
    "SimpleVoxelViewer;Simple Voxel Viewer")
  SET(CPACK_NSIS_MUI_FINISHPAGE_RUN "VoxelCLI.exe")
  SET(CPACK_NSIS_MUI_FINISHPAGE_RUN_TEXT "Run Voxel CLI")
  
  IF(CMAKE_CL_64)
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
  ELSE()
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
  ENDIF()
  
  include(CPack)
ENDIF()