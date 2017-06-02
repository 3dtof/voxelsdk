cmake_minimum_required (VERSION 2.8.13)
project (Voxel)

set(VOXEL_MAJOR_VERSION 0)
set(VOXEL_MINOR_VERSION 6)
set(VOXEL_PATCH_VERSION 9)
set(VOXEL_VERSION ${VOXEL_MAJOR_VERSION}.${VOXEL_MINOR_VERSION}.${VOXEL_PATCH_VERSION})
set(VOXEL_ABI_VERSION 34)
set(VOXEL_CONF_VERSION 1)

### Do not export any symbol by default
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
include(GenerateExportHeader)


set(CMAKE_INCLUDE_CURRENT_DIR 1)

MESSAGE(${CMAKE_SYSTEM_NAME})

set(ARCH ${CMAKE_SYSTEM_PROCESSOR})

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  SET(LINUX 1)
  add_definitions(-msse2 -pthread -std=c++11 -fPIC -ffast-math)
  ADD_DEFINITIONS(-DLINUX)  

  set(COMMON_LIBS ${COMMON_LIBS} usb-1.0 dl)
  SET(COMMON_LIBS_PRIVATE "")
  set(COMMON_INCLUDE
    ${COMMON_INCLUDE}
    /usr/include/libusb-1.0
    Util/
  )
  
  if(ARCH STREQUAL "x86_64")
    set(ARCH amd64)
  endif()
  
  file(WRITE "${CMAKE_BINARY_DIR}/make_deb.sh" "for i in CPackConfig-*.cmake; do cpack -G DEB --config $i; done\n")
  
  SET(COMMON_LIB_FILES)
  
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  SET(WINDOWS 1)
  add_definitions(/fp:fast /W0 /bigobj)
  ADD_DEFINITIONS(-DWINDOWS /wd4251)
  if(NOT MSVC_VERSION GREATER 1800)
    SET(COMMON_LIBS setupapi strmiids strmbase Cfgmgr32 Version)
  else()
    SET(COMMON_LIBS setupapi strmiids strmbase Cfgmgr32 Version legacy_stdio_definitions)
  endif()
  SET(COMMON_LIBS_PRIVATE ${CMAKE_SOURCE_DIR}/Voxel/CyAPI/CyAPI.lib)
  
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/dllversion.rc.in
    ${CMAKE_CURRENT_BINARY_DIR}/dllversion.rc
    @ONLY)
  
  SET(COMMON_LIB_FILES ${CMAKE_CURRENT_BINARY_DIR}/dllversion.rc)
  
endif()

set(CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

if(NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
else()
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${config} c)
    message(${c})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${c} ${PROJECT_BINARY_DIR}/lib)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${c} ${PROJECT_BINARY_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${c} ${PROJECT_BINARY_DIR}/lib)
  endforeach()
endif()
  
include_directories(
  Voxel
  TI3DToF
  VoxelPCL
  ${PROJECT_BINARY_DIR}/Voxel
  ${PROJECT_BINARY_DIR}/TI3DToF
  ${PROJECT_BINARY_DIR}/VoxelPCL
  ${COMMON_INCLUDE}
)

find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

IF(LINUX)
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${ARCH})
  set(CPACK_DEBIAN_PACKAGE_SECTION "Electronics")
  set(CPACK_DEB_COMPONENT_INSTALL 1)
ENDIF()

INCLUDE(config/common.cmake)

find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/config/Doxyfile.in ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )
  install(CODE "execute_process(COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_BINARY_DIR})"
        COMPONENT voxel_dev)
  
  if(LINUX)
    install(DIRECTORY 
      ${CMAKE_BINARY_DIR}/doc/html/ 
      DESTINATION share/doc/voxel-sdk-${VOXEL_VERSION}
      COMPONENT voxel_dev)
  elseif(WINDOWS)
  install(DIRECTORY 
      ${CMAKE_BINARY_DIR}/doc/html/ 
      DESTINATION doc/voxel-sdk
      COMPONENT voxel_dev)
  endif()
endif()
  
#### Packaging related information -- common to both platforms
set(CPACK_PACKAGE_NAME "Voxel SDK")
set(CPACK_PACKAGE_VERSION ${VOXEL_VERSION})
set(CPACK_PACKAGE_CONTACT "prasad.bhat@ti.com")
set(CPACK_PACKAGE_VENDOR "Texas Instruments, Inc.")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library package. Contains the main Voxel library.")
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)

add_subdirectory(Voxel)
add_subdirectory(VoxelPCL)
add_subdirectory(TI3DToF)
add_subdirectory(Test)
add_subdirectory(App)
add_subdirectory(Util)


IF(WINDOWS)
  SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/config)
  SET(CPACK_NSIS_MODIFY_PATH ON)
  
  SET(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
  
  IF(CMAKE_CL_64)
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
  ELSE()
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
  ENDIF()
  
  INCLUDE(config/windows_all_in_installer_config.cmake)
  
  include(CPack)
ENDIF()
