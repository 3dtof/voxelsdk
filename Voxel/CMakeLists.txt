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
  FFT.cpp
  Parameter.cpp
  RegisterProgrammer.cpp
  USBSystem.cpp
  UVC.cpp
  UVCXU.cpp
  Logger.cpp
  Timer.cpp
  Streamer.cpp
  USBIO.cpp
  USBBulkStreamer.cpp
  UVCStreamer.cpp
  CameraSystem.cpp
  DepthCamera.cpp
  FrameStream.cpp
  FrameGenerator.cpp
  DepthCameraLibrary.cpp
  TinyXML2.cpp # for parsing DML files
  ParameterDMLParser.cpp
  Configuration.cpp
  PointCloudFrameGenerator.cpp
  PointCloudTransform.cpp
  Filter/Filter.cpp
  Filter/IIRFilter.cpp
  Filter/MedianFilter.cpp
  Filter/TemporalMedianFilter.cpp
  Filter/SmoothFilter.cpp
  Filter/BilateralFilter.cpp
  Filter/FilterFactory.cpp
  Filter/VoxelFilterFactory.cpp
  Convolve2D.cpp
  ${OS_CPP_FILES}
)

generate_export_header(voxel
  EXPORT_FILE_NAME "VoxelExports.h"
)
  
target_link_libraries(voxel
  LINK_PUBLIC ${COMMON_LIBS}
  LINK_PRIVATE ${COMMON_LIBS_PRIVATE}
)

set_target_properties(voxel PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION})
  
install(TARGETS voxel
  EXPORT VoxelTargets
  LIBRARY DESTINATION lib COMPONENT voxel
  ARCHIVE DESTINATION lib COMPONENT voxel
  RUNTIME DESTINATION lib COMPONENT voxel
)

install(FILES 
  CameraSystem.h
  Common.h
  Configuration.h
  DepthCamera.h
  DepthCameraLibrary.h
  DepthCameraFactory.h
  DownloaderFactory.h
  Device.h
  Downloader.h
  Frame.h
  FrameStream.h
  FrameGenerator.h
  PointCloudFrameGenerator.h
  SerializedObject.h
  Serializable.h
  FrameBuffer.h
  Logger.h
  Parameter.h
  ParameterDMLParser.h
  Point.h
  Ptr.h
  RegisterProgrammer.h
  Streamer.h
  USBBulkStreamer.h
  Timer.h
  TinyXML2.h
  USBSystem.h
  UVC.h
  UVCStreamer.h
  UVCXU.h
  USBIO.h
  VideoMode.h
  PointCloudTransform.h
  Convolve2D.h
  ${CMAKE_CURRENT_BINARY_DIR}/VoxelExports.h
  DESTINATION include/voxel
  COMPONENT voxel_dev
)

install(FILES
  Filter/Filter.h
  Filter/FilterSet.h
  Filter/FilterParameter.h
  Filter/FilterFactory.h
  Filter/IIRFilter.h
  Filter/MedianFilter.h
  Filter/TemporalMedianFilter.h
  Filter/BilateralFilter.h
  Filter/SmoothFilter.h
  Filter/DiscreteGaussian.h
  DESTINATION include/voxel/Filter
  COMPONENT voxel_dev
)

configure_file(VoxelConfig.h.in "${CMAKE_CURRENT_BINARY_DIR}/VoxelConfig.h" @ONLY)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/VoxelConfig.h
  DESTINATION include/voxel
  COMPONENT voxel_dev
)

add_subdirectory(SWIG)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL voxel)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library package. Contains the main Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libusb-1.0-0, libudev1")
  create_cpack_config(libvoxel ${VOXEL_VERSION})

  set(CPACK_COMPONENTS_ALL voxel_dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library developer package. Contains header files needed to compile applications linking to Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (= ${VOXEL_VERSION}), libusb-1.0-0-dev, libudev-dev, linux-libc-dev")
  create_cpack_config(libvoxel-dev ${VOXEL_VERSION})
  
  set(CPACK_COMPONENTS_ALL voxel_python)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Python bindings for Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (= ${VOXEL_VERSION}), python2.7 (>= 2.7.0)")
  create_cpack_config(libvoxel-python ${VOXEL_VERSION})
ENDIF()
