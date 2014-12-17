find_package(PCL 1.6 REQUIRED COMPONENTS common io visualization)

add_definitions(${PCL_DEFINITIONS})

add_library(voxelpcl SHARED PCLGrabber.cpp PCLViewer.cpp)
target_include_directories(voxelpcl PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(voxelpcl voxel ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(SimplePCLViewer SimplePCLViewer.cpp)
target_link_libraries(SimplePCLViewer voxelpcl)

add_executable(VoxelCLI VoxelCLI.cpp CLIManager.cpp LineNoise.cpp)
target_link_libraries(VoxelCLI voxelpcl voxel)

install(TARGETS voxelpcl
  LIBRARY
  DESTINATION lib
  COMPONENT pcl
)

install(TARGETS SimplePCLViewer VoxelCLI
  RUNTIME
  DESTINATION bin
  COMPONENT apps
)

