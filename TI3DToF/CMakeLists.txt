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

### Copy DML and firmware files to build directory
add_custom_command(TARGET ti3dtof PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/TI3DToF/OPT9220_0v27.fw $<TARGET_FILE_DIR:ti3dtof>)
                   
add_custom_command(TARGET ti3dtof PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/TI3DToF/OPT9220A.dml $<TARGET_FILE_DIR:ti3dtof>)