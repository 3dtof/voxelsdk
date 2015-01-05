find_package(PCL 1.6 REQUIRED COMPONENTS common io visualization)

SET(VOXEL_PCL_VERSION ${VOXEL_VERSION})
SET(VOXEL_APP_VERSION ${VOXEL_VERSION})

IF(WINDOWS)
#CMAKE_POLICY(SET CMP0028 OLD)
FIND_PACKAGE(Qt5Widgets REQUIRED)
ENDIF()

add_definitions(${PCL_DEFINITIONS})

add_library(voxelpcl SHARED PCLGrabber.cpp PCLViewer.cpp)
target_include_directories(voxelpcl PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(voxelpcl LINK_PUBLIC voxel LINK_PRIVATE ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

generate_export_header(voxelpcl
  EXPORT_FILE_NAME "VoxelPCLExports.h"
)

add_executable(SimpleVoxelViewer SimpleVoxelViewer.cpp)
target_link_libraries(SimpleVoxelViewer voxelpcl)

add_executable(VoxelCLI VoxelCLI.cpp CLIManager.cpp LineNoise.cpp)
target_link_libraries(VoxelCLI voxelpcl voxel)

install(TARGETS voxelpcl
  EXPORT VoxelPCLTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
  COMPONENT pcl
)

install(FILES
  PCLGrabber.h
  PCLViewer.h
  ${CMAKE_CURRENT_BINARY_DIR}/VoxelPCLExports.h
  DESTINATION include/voxel/pcl
  COMPONENT pcl-dev
)

create_cmake_config(VoxelPCL voxelpcl pcl-dev)

install(TARGETS SimpleVoxelViewer VoxelCLI
  RUNTIME DESTINATION bin
  COMPONENT apps
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL pcl)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "PCL integration with Grabber and Viewer")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION}), libpcl-visualization-1.7, libpcl-io-1.7, libpcl-common-1.7")
  create_cpack_config(libvoxel-pcl ${VOXEL_PCL_VERSION})

  set(CPACK_COMPONENTS_ALL pcl-dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library developer package. Contains header files needed to compile applications linking to Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel-dev (>= ${VOXEL_VERSION}), libpcl-visualization-1.7-dev, libpcl-io-1.7-dev, libpcl-common-1.7-dev")
  create_cpack_config(libvoxel-pcl-dev ${VOXEL_PCL_VERSION})

  set(CPACK_COMPONENTS_ALL apps)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel sample applications")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION}), libvoxel-pcl (>= ${VOXEL_PCL_VERSION})")
  create_cpack_config(libvoxel-apps ${VOXEL_APP_VERSION})
ELSEIF(WINDOWS)
  #Add NSIS related options here
ENDIF()