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
  LIBRARY DESTINATION lib COMPONENT pcl
  ARCHIVE DESTINATION lib COMPONENT pcl
  RUNTIME DESTINATION lib COMPONENT pcl
)

install(FILES
  PCLGrabber.h
  PCLViewer.h
  ${CMAKE_CURRENT_BINARY_DIR}/VoxelPCLExports.h
  DESTINATION include/voxel/pcl
  COMPONENT pcl_dev
)

create_cmake_config(VoxelPCL "voxelpcl;voxel" pcl_dev)

install(TARGETS SimpleVoxelViewer VoxelCLI
  RUNTIME DESTINATION bin
  COMPONENT apps
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL pcl)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "PCL integration with Grabber and Viewer")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION}), libpcl-visualization-1.7, libpcl-io-1.7, libpcl-common-1.7")
  create_cpack_config(libvoxel-pcl ${VOXEL_PCL_VERSION})

  set(CPACK_COMPONENTS_ALL pcl_dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library developer package. Contains header files needed to compile applications linking to Voxel library.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel-dev (>= ${VOXEL_VERSION}), libpcl-visualization-1.7-dev, libpcl-io-1.7-dev, libpcl-common-1.7-dev")
  create_cpack_config(libvoxel-pcl-dev ${VOXEL_PCL_VERSION})

  set(CPACK_COMPONENTS_ALL apps)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel sample applications")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION}), libvoxel-pcl (>= ${VOXEL_PCL_VERSION})")
  create_cpack_config(libvoxel-apps ${VOXEL_APP_VERSION})
ELSEIF(WINDOWS)
  set(CPACK_COMPONENT_PCL_DISPLAY_NAME "Voxel-PCL SDK" PARENT_SCOPE)
  set(CPACK_COMPONENT_PCL_DEV_DISPLAY_NAME "Voxel-PCL SDK C++ headers" PARENT_SCOPE)
  set(CPACK_COMPONENT_APPS_DISPLAY_NAME "Voxel Applications" PARENT_SCOPE)
  set(CPACK_COMPONENT_PCL_DESCRIPTION "Support libraries for PointCloud.org (PCL)" PARENT_SCOPE)
  set(CPACK_COMPONENT_PCL_DEV_DESCRIPTION "C++ headers for PCL support libraries" PARENT_SCOPE)
  set(CPACK_COMPONENT_APPS_DESCRIPTION "Voxel Applications" PARENT_SCOPE)
  
  set(CPACK_COMPONENT_PCL_DEPENDS lib PARENT_SCOPE)
  set(CPACK_COMPONENT_PCL_DEV_DEPENDS lib_dev PARENT_SCOPE)
  set(CPACK_COMPONENT_PCL_DEV_DEPENDS pcl PARENT_SCOPE)
  set(CPACK_COMPONENT_APPS_DEPENDS pcl PARENT_SCOPE)
  
  set(CPACK_COMPONENT_PCL_GROUP "VoxelPCL" PARENT_SCOPE)
  set(CPACK_COMPONENT_PCL_DEV_GROUP "VoxelPCL" PARENT_SCOPE)
  SET(CPACK_COMPONENT_GROUP_VOXELPCL_DESCRIPTION "Voxel-PCL Integration SDK library and headers" PARENT_SCOPE)
ENDIF()