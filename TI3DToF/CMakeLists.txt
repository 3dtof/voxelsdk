add_library(ti3dtof SHARED
  VoxelXUProgrammer.cpp
  ToFCamera.cpp
  ToFHaddockCamera.cpp
  Voxel14Camera.cpp
  ToFCameraFactory.cpp
)

target_link_libraries(ti3dtof
  voxel
)
  
install(TARGETS ti3dtof
  LIBRARY
  DESTINATION lib
  COMPONENT libraries
)