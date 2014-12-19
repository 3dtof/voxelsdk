add_executable(DeviceTest DeviceTest.cpp)
target_link_libraries(DeviceTest voxel)

add_executable(DownloaderTest DownloaderTest.cpp)
target_link_libraries(DownloaderTest voxel)

add_executable(Voxel14RegisterTest Voxel14RegisterTest.cpp)
target_link_libraries(Voxel14RegisterTest ti3dtof)

add_executable(UVCStreamerTest UVCStreamerTest.cpp)
target_link_libraries(UVCStreamerTest voxel)

add_executable(CameraSystemTest CameraSystemTest.cpp)
target_link_libraries(CameraSystemTest voxel ti3dtof)

add_executable(DMLParseTest DMLParseTest.cpp)
target_link_libraries(DMLParseTest voxel ti3dtof)

install(TARGETS
  DeviceTest 
  DownloaderTest 
  Voxel14RegisterTest 
  UVCStreamerTest
  CameraSystemTest
  DMLParseTest
  RUNTIME
  DESTINATION bin
  COMPONENT test
)
                       
set(CPACK_COMPONENTS_ALL test)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library test programs.")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= 0.1), libti3dtof (>= 0.1)")
create_cpack_config(libvoxel-test 0.1)
