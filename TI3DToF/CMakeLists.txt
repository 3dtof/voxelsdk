SET(TI3DToF_VERSION ${VOXEL_VERSION} PARENT_SCOPE)

add_library(ti3dtof SHARED
  VoxelXUProgrammer.cpp
  ToFCamera.cpp
  ToFHaddockCamera.cpp
  Voxel14Camera.cpp
  ToFCameraFactory.cpp
)

generate_export_header(ti3dtof
  EXPORT_FILE_NAME "TI3DToFExports.h"
)

target_link_libraries(ti3dtof
  voxel
)
  
### Copy DML and firmware files to build directory
add_custom_command(TARGET ti3dtof PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/TI3DToF/OPT9220_0v27.fw $<TARGET_FILE_DIR:ti3dtof>)
                   
add_custom_command(TARGET ti3dtof PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/TI3DToF/OPT9220A.dml $<TARGET_FILE_DIR:ti3dtof>)
                   
                   
install(TARGETS ti3dtof
  EXPORT TI3DToFTargets
  LIBRARY DESTINATION lib/voxel
  ARCHIVE DESTINATION lib/voxel
  RUNTIME DESTINATION lib/voxel
  COMPONENT ti3dtof_lib
)

IF(LINUX)
install(FILES 
  OPT9220A.dml
  DESTINATION /etc/voxel/
  COMPONENT ti3dtof_lib
)

install(FILES 
  OPT9220_0v27.fw
  DESTINATION /lib/firmware/voxel
  COMPONENT ti3dtof_lib
)

install(FILES
  72-ti3dtof.rules
  DESTINATION /etc/udev/rules.d/
  COMPONENT ti3dtof_lib
)
ELSEIF(WINDOWS)
INSTALL(FILES
  OPT9220A.dml
  DESTINATION lib/voxel
  COMPONENT ti3dtof_lib)

INSTALL(FILES
  OPT9220_0v27.fw
  DESTINATION lib/voxel
  COMPONENT ti3dtof_lib)
ENDIF()

install(FILES
  ToFCamera.h
  ToFCameraFactory.h
  ToFHaddockCamera.h
  Voxel14Camera.h
  VoxelXUProgrammer.h
  ${CMAKE_CURRENT_BINARY_DIR}/TI3DToFExports.h
  DESTINATION include/voxel/ti3dtof
  COMPONENT ti3dtof_dev
)

create_cmake_config(TI3DToF ti3dtof ti3dtof_dev)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL ti3dtof_lib)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Support libraries for ToF depth camera from Texas Instruments")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/debian/postrm;")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION})")
  create_cpack_config(libti3dtof ${TI3DToF_VERSION})

  set(CPACK_COMPONENTS_ALL ti3dtof_dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Header files for ToF depth camera from Texas Instruments.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel-dev (>= ${VOXEL_VERSION})")
  create_cpack_config(libti3dtof-dev ${TI3DToF_VERSION})
ELSEIF(WINDOWS)
  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\ti3dtof.lib` `C:\\\\Program Files\\\\VoxelCommon\\\\lib`\n")
  
  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\ti3dtof.dll` `C:\\\\Program Files\\\\VoxelCommon\\\\lib`\n")
  
  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\OPT9220A.dml` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n")
  
  #Add PARENT_SCOPE only to the last in the sequence of these commands
  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\OPT9220_0v27.fw` `C:\\\\Program Files\\\\VoxelCommon\\\\fw`\n")
  
  SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n CreateShortCut `\\\$SMPROGRAMS\\\\\\\$STARTMENU_FOLDER\\\\Open CMD Prompt.lnk` `C:\\\\Windows\\\\System32\\\\cmd.exe`\n" PARENT_SCOPE)
  
  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}\n \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\lib` `ti3dtof.lib`\n")
  
  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}\n \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\lib` `ti3dtof.dll`\n")
  
  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}\n \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `OPT9220A.dml`\n")
  
  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}\n \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\fw` `OPT9220_0v27.fw`\n")
  
  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}\n RMDir /REBOOTOK `C:\\\\Program Files\\\\VoxelCommon`\n")
  
  SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS
    "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}\n \\\${MenuItemRemove} `Open CMD Prompt.lnk`\n" PARENT_SCOPE)

  set(CPACK_COMPONENT_TI3DTOF_LIB_DISPLAY_NAME "TI3DToF Voxel library" PARENT_SCOPE)
  set(CPACK_COMPONENT_TI3DTOF_DEV_DISPLAY_NAME "TI3DToF C++ headers" PARENT_SCOPE)
  
  set(CPACK_COMPONENT_TI3DTOF_LIB_DESCRIPTION "TI 3D Depth Camera Support Libraries" PARENT_SCOPE)
  set(CPACK_COMPONENT_TI3DTOF_DEV_DESCRIPTION "C++ headers for TI 3D Depth Camera Support Libraries" PARENT_SCOPE)
  
  set(CPACK_COMPONENT_TI3DTOF_LIB_GROUP "TI3DToF" PARENT_SCOPE)
  set(CPACK_COMPONENT_TI3DTOF_DEV_GROUP "TI3DToF" PARENT_SCOPE)
  SET(CPACK_COMPONENT_GROUP_TI3DTOF_DESCRIPTION "TI ToF camera related support libraries and C++ headers" PARENT_SCOPE)
ENDIF()