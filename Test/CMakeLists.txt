SET(VOXEL_TEST_VERSION ${VOXEL_VERSION} PARENT_SCOPE)

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
             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL test)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library test programs.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION}), libti3dtof (>= ${TI3DToF_VERSION})")
  create_cpack_config(libvoxel-test ${VOXEL_TEST_VERSION})
ELSEIF(WINDOWS)
  set(CPACK_COMPONENT_TEST_DESCRIPTION "Voxel Test Applications" PARENT_SCOPE)
  set(CPACK_COMPONENT_TEST_DISPLAY_NAME "Voxel Test" PARENT_SCOPE)
ENDIF()
