cmake_minimum_required (VERSION 2.8.11)
project (libvoxelapps)

set(VOXEL_APP_MAJOR_VERSION 0)
set(VOXEL_APP_MINOR_VERSION 4)
set(VOXEL_APP_PATCH_VERSION 8)
set(VOXEL_APP_VERSION ${VOXEL_APP_MAJOR_VERSION}.${VOXEL_APP_MINOR_VERSION}.${VOXEL_APP_PATCH_VERSION})
set(VOXEL_VERSION ${VOXEL_APP_VERSION})
set(VOXEL_PCL_VERSION ${VOXEL_APP_VERSION})

### Do not export any symbol by default
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
include(GenerateExportHeader)


set(CMAKE_INCLUDE_CURRENT_DIR 1)

MESSAGE(${CMAKE_SYSTEM_NAME})

set(ARCH ${CMAKE_SYSTEM_PROCESSOR})

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  SET(LINUX 1)
  add_definitions(-msse2 -pthread -std=c++11)
  ADD_DEFINITIONS(-DLINUX)  
  
  if(ARCH STREQUAL "x86_64")
    set(ARCH amd64)
  endif()
  
  file(WRITE "${CMAKE_BINARY_DIR}/make_deb.sh" "for i in CPackConfig-*.cmake; do cpack -G DEB --config $i; done\n")
  
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  SET(WINDOWS 1)
  ADD_DEFINITIONS(-DWINDOWS /wd4251)
endif()

find_package(VoxelPCL ${VOXEL_PCL_VERSION} REQUIRED)

message(${VOXEL_PCL_INCLUDE_DIRS})

include_directories(
  ${VOXEL_INCLUDE_DIRS}
  ${VOXEL_PCL_INCLUDE_DIRS}
)

find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

IF(LINUX)
  function(create_cpack_config name ver)
    set(CPACK_PACKAGE_VERSION "${ver}")
    set(CPACK_PACKAGE_NAME "${name}")
    set(CPACK_PACKAGE_FILE_NAME "${name}-${ver}-${ARCH}")
    set(CPACK_OUTPUT_CONFIG_FILE "${CMAKE_BINARY_DIR}/CPackConfig-${name}.cmake")
    include(CPack)
  endfunction(create_cpack_config)
  
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${ARCH})
  set(CPACK_DEBIAN_PACKAGE_SECTION "Electronics")
  set(CPACK_DEB_COMPONENT_INSTALL 1)
ENDIF()

find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )
  install(CODE "execute_process(COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_BINARY_DIR})"
    COMPONENT apps)
  
  if(LINUX)
    install(DIRECTORY 
      ${CMAKE_BINARY_DIR}/doc/html/ 
      DESTINATION share/doc/voxelapps-${VOXEL_PCL_VERSION}
      COMPONENT apps)
  elseif(WINDOWS)
  install(DIRECTORY 
      ${CMAKE_BINARY_DIR}/doc/html/ 
      DESTINATION doc/voxelapps
      COMPONENT apps)
  endif()
endif()
  
#### Packaging related information -- common to both platforms
set(CPACK_PACKAGE_NAME "Voxel SDK")
set(CPACK_PACKAGE_VERSION ${VOXEL_APP_VERSION})
set(CPACK_PACKAGE_CONTACT "prasad.bhat@ti.com")
set(CPACK_PACKAGE_VENDOR "Texas Instruments, Inc.")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library package. Contains the main Voxel library.")
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)

add_subdirectory(App)
INCLUDE(windows_all_in_installer_config.cmake)

IF(WINDOWS)
  SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
  SET(CPACK_NSIS_MODIFY_PATH ON)
  
  SET(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
  
  IF(CMAKE_CL_64)
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
  ELSE()
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
  ENDIF()
  
  include(CPack)
ENDIF()
