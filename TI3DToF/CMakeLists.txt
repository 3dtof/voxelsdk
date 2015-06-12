add_library(ti3dtof SHARED
  VoxelProgrammerBase.cpp
  VoxelUSBProgrammer.cpp
  VoxelXUProgrammer.cpp
  ToFCameraBase.cpp
  ToFCamera.cpp
  ToFDepthFrameGenerator.cpp
  ToFCameraFactoryBase.cpp
  ToFFrameGenerator.cpp
  ToFCalculusCamera.cpp
  ToFHaddockCamera.cpp
  ToFTintinCamera.cpp
  ToFCrossTalkFilter.cpp
)

generate_export_header(ti3dtof
  EXPORT_FILE_NAME "TI3DToFExports.h"
)

target_link_libraries(ti3dtof
  voxel ${COMMON_LIBS}
)

add_subdirectory(boards)

list(APPEND CONF_FILES
  OPT8320.dml
  OPT9220.dml
  OPT9221.dml
)

list(APPEND FW_FILES
  OPT9220_0v27.tip
  OPT9221_0v8.tip
)

set(filesToCopy) # Remove this variable
  
### Copy configuration and firmware files to build directory
foreach(f ${CONF_FILES} ${FW_FILES})
  get_filename_component(filename ${f} NAME)
  list(APPEND filesToCopy
    ${CMAKE_CURRENT_BINARY_DIR}/${filename}
  )
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${filename}
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_CURRENT_SOURCE_DIR}/${f} ${CMAKE_CURRENT_BINARY_DIR}/
                   MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/${f})
endforeach()

if(WINDOWS)
list(APPEND filesToCopy
    ${CMAKE_CURRENT_BINARY_DIR}/WinDrivers
  )
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/WinDrivers
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR}/WinDrivers ${CMAKE_CURRENT_BINARY_DIR}/WinDrivers
                   MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/WinDrivers)
                   
install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/WinDrivers
  DESTINATION Prerequisites
  COMPONENT ti3dtof
)
endif()

add_custom_target(filesToCopyTarget ALL DEPENDS ${filesToCopy})
                   
install(TARGETS ti3dtof
  EXPORT TI3DToFTargets
  LIBRARY DESTINATION lib COMPONENT ti3dtof
  ARCHIVE DESTINATION lib COMPONENT ti3dtof
  RUNTIME DESTINATION lib COMPONENT ti3dtof
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
  ${UDEV_FILES}
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
  ToFCalculusCamera.h
  ToFCamera.h
  ToFCameraBase.h
  ToFCameraFactoryBase.h
  ToFHaddockCamera.h
  ToFCalculusCamera.h
  ToFTintinCamera.h
  ToFDepthFrameGenerator.h
  ToFCrossTalkFilter.h
  ToFFrameGenerator.h
  ToFDepthFrameGenerator.h
  VoxelProgrammerBase.h
  VoxelUSBProgrammer.h
  VoxelXUProgrammer.h
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
