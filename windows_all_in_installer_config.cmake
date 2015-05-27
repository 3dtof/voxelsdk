IF(CMAKE_CL_64)
  install(PROGRAMS Windows/vcredist_x64.exe DESTINATION Prerequistes)
ELSE()
  install(PROGRAMS Windows/vcredist_x86.exe DESTINATION Prerequistes)
ENDIF()

IF(CMAKE_CL_64)
  SET(XARCHSUFFIX "x64")
  SET(ARCHSUFFIX "64")
ELSE()
  SET(XARCHSUFFIX "x86")
  SET(ARCHSUFFIX "32")
ENDIF() 

SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS 
  "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n
  \\\${If}  \\\${SectionIsSelected} \\\${voxel}\n
    ExecWait '\\\"$INSTDIR\\\\Prerequistes\\\\vcredist_${XARCHSUFFIX}.exe\\\" /install'\n
  \\\${EndIf}\n

  \\\${If}  \\\${SectionIsSelected} \\\${voxel_python}\n
;    messagebox mb_ok \\\"\\\$INSTDIR\\\\lib\\\\python2.7\\\\* \\\$PythonDir\\\\Lib\\\\site-packages\\\"\n
    CopyFiles \\\"\\\$INSTDIR\\\\lib\\\\python2.7\\\\_Voxel.pyd\\\" \\\"\\\$PythonDir\\\\Lib\\\\site-packages\\\\\\\"\n
    CopyFiles \\\"\\\$INSTDIR\\\\lib\\\\python2.7\\\\Voxel.py\\\" \\\"\\\$PythonDir\\\\Lib\\\\site-packages\\\\\\\"\n
  \\\${EndIf}\n
      
  \\\${If}  \\\${SectionIsSelected} \\\${ti3dtof_lib}\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\ti3dtof.lib` `C:\\\\Program Files\\\\VoxelCommon\\\\lib`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\ti3dtof.dll` `C:\\\\Program Files\\\\VoxelCommon\\\\lib`\n 
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\OPT8320.dml` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\OPT9220.dml` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\CalculusCDKCamera.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\CalculusCDKCameraNormal.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\HaddockCDKCamera.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\HaddockCDKCameraNormal.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\Voxel14Camera.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\Voxel14CameraNormal.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\Voxel14CameraTestMode.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\Voxel14CameraPhaseOffset.bin` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\VoxelDCamera.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\TintinCDKCamera.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\TintinCDKCameraShortRange.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\TintinCDKCameraLongRange.conf` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\TintinCDKCameraPhaseOffset.bin` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\fw\\\\OPT9220_0v27.tip` `C:\\\\Program Files\\\\VoxelCommon\\\\fw`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\conf\\\\OPT9221.dml` `C:\\\\Program Files\\\\VoxelCommon\\\\conf`\n
    \\\${FileCopy} `\\\$INSTDIR\\\\lib\\\\voxel\\\\fw\\\\OPT9221_0v8.tip` `C:\\\\Program Files\\\\VoxelCommon\\\\fw`\n
    ExecWait '\\\"$INSTDIR\\\\Prerequistes\\\\Drivers\\\\TI3DToF\\\\dpinst${ARCHSUFFIX}.exe\\\" /SW /PATH \\\"\\\$INSTDIR\\\\Prerequistes\\\\Drivers\\\\TI3DToF\\\\${XARCHSUFFIX}\\\"'\n
  \\\${EndIf}\n
  "
)
  
SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS 
  "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\lib` `ti3dtof.lib`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\lib` `ti3dtof.dll`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `CalculusCDKCamera.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `CalculusCDKCameraNormal.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `HaddockCDKCamera.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `HaddockCDKCameraNormal.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `Voxel14Camera.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `Voxel14CameraNormal.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `Voxel14CameraTestMode.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `Voxel14CameraPhaseOffset.bin`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `VoxelDCamera.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `TintinCDKCamera.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `TintinCDKCameraShortRange.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `TintinCDKCameraLongRange.conf`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `TintinCDKCameraPhaseOffset.bin`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `OPT8320.dml`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `OPT9220.dml`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\fw` `OPT9220_0v27.tip`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\conf` `OPT9221.dml`\n
  \\\${FileRemove} `C:\\\\Program Files\\\\VoxelCommon\\\\fw` `OPT9221_0v8.tip`\n
  RMDir /REBOOTOK `C:\\\\Program Files\\\\VoxelCommon`\n

  !insertmacro un.getPythonPath

  \\\${If} \\\$PythonDir != \\\"\\\"
    Delete \\\"\\\$PythonDir\\\\Lib\\\\site-packages\\\\_Voxel.pyd\\\"\n
    Delete \\\"\\\$PythonDir\\\\Lib\\\\site-packages\\\\Voxel.py\\\"\n
  \\\${EndIf}
  

  !insertmacro MUI_STARTMENU_GETFOLDER Application $MUI_TEMP\n
  Delete '\$SMPROGRAMS\\\\$MUI_TEMP\\\\Voxel CLI.lnk'\n
  Delete '\$SMPROGRAMS\\\\$MUI_TEMP\\\\Simple Voxel Viewer.lnk'\n
  Delete '\$SMPROGRAMS\\\\$MUI_TEMP\\\\CMD in Voxel Directory.lnk'\n
  "
)

set(CPACK_NSIS_CREATE_ICONS_EXTRA 
    "\\\${If}  \\\${SectionIsSelected} \\\${apps}\n
        CreateShortCut '\$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Voxel CLI.lnk' '\$INSTDIR\\\\bin\\\\VoxelCLIStart.cmd'\n
        CreateShortCut '\$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Simple Voxel Viewer.lnk' '\$INSTDIR\\\\bin\\\\SimpleVoxelViewer.exe'\n
        CreateShortCut '\$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\CMD in Voxel Directory.lnk' 'C:\\\\Windows\\\\System32\\\\cmd.exe'\n
     \\\${EndIf}\n")


set(CPACK_COMPONENT_VOXEL_DISPLAY_NAME "Voxel Libraries")
set(CPACK_COMPONENT_VOXEL_DESCRIPTION "Core Voxel Libraries")
set(CPACK_COMPONENT_VOXEL_DEV_DISPLAY_NAME "Voxel C++ Headers")
set(CPACK_COMPONENT_VOXEL_DEV_DESCRIPTION "C++ Headers for Core Voxel Libraries")
set(CPACK_COMPONENT_TI3DTOF_DISPLAY_NAME "TI3DToF Voxel library")
set(CPACK_COMPONENT_TI3DTOF_DESCRIPTION "TI 3D Depth Camera Support Libraries")
set(CPACK_COMPONENT_TI3DTOF_DEV_DISPLAY_NAME "TI3DToF C++ Headers")
set(CPACK_COMPONENT_TI3DTOF_DEV_DESCRIPTION "C++ Headers for TI 3D Depth Camera Support Libraries")
set(CPACK_COMPONENT_VOXEL_PYTHON_DISPLAY_NAME "Voxel Python Bindings")
set(CPACK_COMPONENT_VOXEL_PYTHON_DESCRIPTION "Python Bindings for Core Voxel libraries")
set(CPACK_COMPONENT_VOXELPCL_DISPLAY_NAME "Voxel-PCL Libraries")
set(CPACK_COMPONENT_VOXELPCL_DESCRIPTION "Voxel support libraries for PointCloud.org (PCL)")
set(CPACK_COMPONENT_VOXELPCL_DEV_DISPLAY_NAME "Voxel-PCL C++ Headers")
set(CPACK_COMPONENT_VOXELPCL_DEV_DESCRIPTION "C++ Headers for Voxel support libraries for PointCloud.org (PCL)")
set(CPACK_COMPONENT_APPS_DISPLAY_NAME "Voxel Applications")
set(CPACK_COMPONENT_APPS_DESCRIPTION "Voxel Applications")

set(CPACK_COMPONENT_APPS_DEPENDS voxel)
set(CPACK_COMPONENT_APPS_DEPENDS voxelpcl)
set(CPACK_COMPONENT_VOXEL_DEV_DEPENDS voxel)
set(CPACK_COMPONENT_VOXELPCL_DEPENDS voxel)
set(CPACK_COMPONENT_VOXELPCL_DEV_DEPENDS voxelpcl)
set(CPACK_COMPONENT_TI3DTOF_DEPENDS voxel)
set(CPACK_COMPONENT_TI3DTOF_DEV_DEPENDS ti3dtof)
