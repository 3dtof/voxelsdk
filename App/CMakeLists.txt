find_package(PCL 1.6 REQUIRED COMPONENTS common io visualization)

add_definitions(${PCL_DEFINITIONS})

add_library(voxelpcl SHARED PCLGrabber.cpp)
target_include_directories(voxelpcl PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(voxelpcl voxel ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(SimplePCLViewer SimplePCLViewer.cpp)
target_include_directories(SimplePCLViewer PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(SimplePCLViewer voxelpcl ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})


install(TARGETS voxelpcl
  LIBRARY
  DESTINATION lib
  COMPONENT pcl
)

install(TARGETS SimplePCLViewer
  RUNTIME
  DESTINATION bin
  COMPONENT apps
)

