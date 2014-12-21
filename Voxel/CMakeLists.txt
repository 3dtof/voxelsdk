IF(LINUX)
  SET(OS_CPP_FILES 
    USBSystemPrivateLinux.cpp
    UVCPrivateLinux.cpp)
ELSEIF(WINDOWS)
  SET(OS_CPP_FILES 
    USBSystemPrivateWindows.cpp
    UVCPrivateWindows.cpp)
ENDIF()

add_library(voxel SHARED
  Device.cpp
  Common.cpp
  Downloader.cpp
  Parameter.cpp
  RegisterProgrammer.cpp
  USBSystem.cpp
  UVC.cpp
  UVCXU.cpp
  Logger.cpp
  Time.cpp
  Streamer.cpp
  UVCStreamer.cpp
  CameraSystem.cpp
  DepthCamera.cpp
  DepthCameraLibrary.cpp
  TinyXML2.cpp # for parsing DML files
  ParameterDMLParser.cpp
  Configuration.cpp
  ${OS_CPP_FILES}
)
  
target_link_libraries(voxel
  ${COMMON_LIBS}
)
  
install(TARGETS voxel
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  COMPONENT lib
)

install(FILES 
  CameraSystem.h
  Common.h
  Configuration.h
  DepthCamera.h
  DepthCameraLibrary.h
  DepthCameraFactory.h
  Device.h
  Downloader.h
  Filter.h
  Frame.h
  FrameBuffer.h
  Logger.h
  Parameter.h
  ParameterDMLParser.h
  Point.h
  Ptr.h
  RegisterProgrammer.h
  Streamer.h
  Time.h
  TinyXML2.h
  USBSystem.h
  UVC.h
  UVCStreamer.h
  UVCXU.h
  VideoMode.h
  DESTINATION include/voxel
  COMPONENT lib-dev
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL lib)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library package. Contains the main Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libusb-1.0-0, libudev1")
  create_cpack_config(libvoxel 0.1)

  set(CPACK_COMPONENTS_ALL lib-dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library developer package. Contains header files needed to compile applications linking to Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (= 0.1), libusb-1.0-0-dev, libudev-dev, linux-libc-dev")
  create_cpack_config(libvoxel-dev 0.1)
ELSEIF(WINDOWS)
  # Added NSIS related options here
ENDIF()