find_package(PCL 1.6 REQUIRED COMPONENTS common io visualization)

add_definitions(${PCL_DEFINITIONS})

add_library(voxelpcl SHARED PCLGrabber.cpp PCLViewer.cpp)
target_include_directories(voxelpcl PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(voxelpcl voxel ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

generate_export_header(voxelpcl
  EXPORT_FILE_NAME "VoxelPCLExports.h"
)

add_executable(SimpleVoxelViewer SimpleVoxelViewer.cpp)
target_link_libraries(SimpleVoxelViewer voxelpcl)

add_executable(VoxelCLI VoxelCLI.cpp CLIManager.cpp LineNoise.cpp)
target_link_libraries(VoxelCLI voxelpcl voxel)

install(TARGETS voxelpcl
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  COMPONENT pcl
)

install(FILES
  PCLGrabber.h
  PCLViewer.h
  ${CMAKE_CURRENT_BINARY_DIR}/VoxelPCLExports.h
  DESTINATION include/voxel/pcl
  COMPONENT pcl-dev
)

install(TARGETS SimpleVoxelViewer VoxelCLI
  RUNTIME
  DESTINATION bin
  COMPONENT apps
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL pcl)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "PCL integration with Grabber and Viewer")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= 0.1), libpcl-visualization-1.7, libpcl-io-1.7, libpcl-common-1.7")
  create_cpack_config(libvoxel-pcl 0.1)

  set(CPACK_COMPONENTS_ALL pcl-dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library developer package. Contains header files needed to compile applications linking to Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel-dev (>= 0.1), libpcl-visualization-1.7-dev, libpcl-io-1.7-dev, libpcl-common-1.7-dev")
  create_cpack_config(libvoxel-pcl-dev 0.1)

  set(CPACK_COMPONENTS_ALL apps)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel sample applications")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= 0.1), libvoxel-pcl (>= 0.1)")
  create_cpack_config(libvoxel-apps 0.1)
ELSEIF(WINDOWS)
  #Add NSIS related options here
ENDIF()