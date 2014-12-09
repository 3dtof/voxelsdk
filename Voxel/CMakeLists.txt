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
  )
  
target_link_libraries(voxel
  ${COMMON_LIBS}
  )
  
install(TARGETS voxel
  LIBRARY
  DESTINATION lib
  COMPONENT libraries
)