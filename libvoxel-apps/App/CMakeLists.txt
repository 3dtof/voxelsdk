IF(WINDOWS)
#CMAKE_POLICY(SET CMP0028 OLD)
FIND_PACKAGE(Qt5Widgets REQUIRED)
ENDIF()

add_executable(SimpleVoxelViewer SimpleVoxelViewer.cpp PCLViewer.cpp)
target_link_libraries(SimpleVoxelViewer ${VOXEL_PCL_LIBRARIES})

add_executable(VoxelCLI VoxelCLI.cpp CLIManager.cpp LineNoise.cpp PCLViewer.cpp)
target_link_libraries(VoxelCLI ${VOXEL_PCL_LIBRARIES})

install(TARGETS SimpleVoxelViewer VoxelCLI
  RUNTIME DESTINATION bin
  COMPONENT apps
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL apps)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel sample applications")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION}), libvoxel-pcl (>= ${VOXEL_PCL_VERSION})")
  create_cpack_config(libvoxel-apps ${VOXEL_APP_VERSION})
ELSEIF(WINDOWS)
  
  INSTALL(PROGRAMS VoxelCLIStart.cmd
    DESTINATION bin
    COMPONENT apps
  )
ENDIF()
