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
  Timer.cpp
  Streamer.cpp
  UVCStreamer.cpp
  CameraSystem.cpp
  DepthCamera.cpp
  DepthCameraLibrary.cpp
  TinyXML2.cpp # for parsing DML files
  ParameterDMLParser.cpp
  Configuration.cpp
  PointCloudTransform.cpp
  Filter/Filter.cpp
  Filter/IIRFilter.cpp
  Filter/FilterFactory.cpp
  Filter/VoxelFilterFactory.cpp
  ${OS_CPP_FILES}
)

generate_export_header(voxel
  EXPORT_FILE_NAME "VoxelExports.h"
)
  
target_link_libraries(voxel
  LINK_PUBLIC ${COMMON_LIBS}
  LINK_PRIVATE ${COMMON_LIBS_PRIVATE}
)
  
install(TARGETS voxel
  EXPORT VoxelTargets
  LIBRARY DESTINATION lib COMPONENT lib
  ARCHIVE DESTINATION lib COMPONENT lib
  RUNTIME DESTINATION lib COMPONENT lib
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
  Frame.h
  FrameBuffer.h
  Logger.h
  Parameter.h
  ParameterDMLParser.h
  Point.h
  Ptr.h
  RegisterProgrammer.h
  Streamer.h
  Timer.h
  TinyXML2.h
  USBSystem.h
  UVC.h
  UVCStreamer.h
  UVCXU.h
  VideoMode.h
  PointCloudTransform.h
  ${CMAKE_CURRENT_BINARY_DIR}/VoxelExports.h
  DESTINATION include/voxel
  COMPONENT lib_dev
)

install(FILES
  Filter/Filter.h
  Filter/FilterParameter.h
  Filter/FilterFactory.h
  Filter/IIRFilter.h
  Filter/MedianFilter.h
  Filter/TemporalMedianFilter.h
  Filter/BilateralFilter.h
  Filter/SmoothFilter.h
  DESTINATION include/voxel/Filter
  COMPONENT lib_dev
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL lib)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library package. Contains the main Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libusb-1.0-0, libudev1")
  create_cpack_config(libvoxel ${VOXEL_VERSION})

  set(CPACK_COMPONENTS_ALL lib_dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library developer package. Contains header files needed to compile applications linking to Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (= ${VOXEL_VERSION}), libusb-1.0-0-dev, libudev-dev, linux-libc-dev")
  create_cpack_config(libvoxel-dev ${VOXEL_VERSION})
ELSEIF(WINDOWS)
  set(CPACK_COMPONENT_LIB_DISPLAY_NAME "Libraries" PARENT_SCOPE)
  set(CPACK_COMPONENT_LIB_DEV_DISPLAY_NAME "C++ Headers" PARENT_SCOPE)
  set(CPACK_COMPONENT_LIB_DESCRIPTION "Core Voxel Libraries" PARENT_SCOPE)
  set(CPACK_COMPONENT_LIB_DEV_DESCRIPTION "C++ headers for Core Voxel Libraries" PARENT_SCOPE)
  
  set(CPACK_COMPONENT_LIB_DEV_DEPENDS lib PARENT_SCOPE)
  
  set(CPACK_COMPONENT_LIB_GROUP "VoxelSDK" PARENT_SCOPE)
  set(CPACK_COMPONENT_LIB_DEV_GROUP "VoxelSDK" PARENT_SCOPE)
  SET(CPACK_COMPONENT_GROUP_VOXELSDK_DESCRIPTION "Voxel SDK library and headers" PARENT_SCOPE)
ENDIF()

create_cmake_config(Voxel voxel lib_dev)