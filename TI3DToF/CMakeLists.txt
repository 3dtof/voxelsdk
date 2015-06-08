add_library(ti3dtof SHARED
  VoxelProgrammerBase.cpp
  VoxelUSBProgrammer.cpp
  VoxelXUProgrammer.cpp
  TintinEEPROMDownloader.cpp
  ToFDownloaderFactory.cpp
  ToFCameraBase.cpp
  ToFCamera.cpp
  ToFDepthFrameGenerator.cpp
  ToFFrameGenerator.cpp
  ToFHaddockCamera.cpp
  ToFTinTinCamera.cpp
  Voxel14Camera.cpp
  HaddockCDKCamera.cpp
  VoxelDCamera.cpp
  TintinCDKCamera.cpp
  ToFCameraFactory.cpp
  ToFCrossTalkFilter.cpp
)

generate_export_header(ti3dtof
  EXPORT_FILE_NAME "TI3DToFExports.h"
)

target_link_libraries(ti3dtof
  voxel ${COMMON_LIBS}
)

set(CONF_FILES
  OPT9220.dml
  OPT9221.dml
  HaddockCDKCamera.conf
  HaddockCDKCameraNormal.conf
  Voxel14Camera.conf
  Voxel14CameraNormal.conf
  Voxel14CameraTestMode.conf
  Voxel14CameraPhaseOffset.bin
  VoxelDCamera.conf
  VoxelDCameraNoCalibration.conf
  TintinCDKCamera.conf
  TintinCDKCameraShortRange.conf
  TintinCDKCameraLongRange.conf
  TintinCDKCameraNoCalibration.conf
  TintinCDKCameraPhaseOffset.bin
)

set(FW_FILES
  OPT9220_0v27.tip
  OPT9221_0v8.tip
)
  
### Copy configuration and firmware files to build directory
foreach(f ${CONF_FILES} ${FW_FILES})
  add_custom_command(TARGET ti3dtof PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/TI3DToF/${f} $<TARGET_FILE_DIR:ti3dtof>)
endforeach()
                   
install(TARGETS ti3dtof
  EXPORT TI3DToFTargets
  LIBRARY DESTINATION lib/voxel COMPONENT ti3dtof
  ARCHIVE DESTINATION lib/voxel COMPONENT ti3dtof
  RUNTIME DESTINATION lib/voxel COMPONENT ti3dtof
)

IF(LINUX)
install(FILES 
  ${CONF_FILES}
  DESTINATION /etc/voxel/
  COMPONENT ti3dtof
)

install(FILES 
  ${FW_FILES}
  DESTINATION /lib/firmware/voxel
  COMPONENT ti3dtof
)

install(FILES
  72-ti3dtof.rules
  DESTINATION /etc/udev/rules.d/
  COMPONENT ti3dtof
)
ELSEIF(WINDOWS)
INSTALL(FILES
  ${CONF_FILES}
  DESTINATION lib/voxel/conf
  COMPONENT ti3dtof)

INSTALL(FILES
  ${FW_FILES}
  DESTINATION lib/voxel/fw
  COMPONENT ti3dtof)
ENDIF()

install(FILES
  ToFCamera.h
  ToFCameraBase.h
  ToFCameraFactory.h
  ToFHaddockCamera.h
  ToFDepthFrameGenerator.h
  ToFCrossTalkFilter.h
  ToFFrameGenerator.h
  HaddockCDKCamera.h
  Voxel14Camera.h
  VoxelDCamera.h
  TintinCDKCamera.h
  VoxelProgrammerBase.h
  VoxelUSBProgrammer.h
  VoxelXUProgrammer.h
  TintinEEPROMDownloader.h
  ToFDownloaderFactory.h
  ${CMAKE_CURRENT_BINARY_DIR}/TI3DToFExports.h
  DESTINATION include/voxel/ti3dtof
  COMPONENT ti3dtof_dev
)

set_target_properties(ti3dtof PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION})

IF(LINUX)
  set(CPACK_COMPONENTS_ALL ti3dtof)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Support libraries for ToF depth camera from Texas Instruments")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/debian/postrm;")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION})")
  create_cpack_config(libti3dtof ${VOXEL_VERSION})

  set(CPACK_COMPONENTS_ALL ti3dtof_dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Header files for ToF depth camera from Texas Instruments.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel-dev (>= ${VOXEL_VERSION})")
  create_cpack_config(libti3dtof-dev ${VOXEL_VERSION})
ENDIF()
