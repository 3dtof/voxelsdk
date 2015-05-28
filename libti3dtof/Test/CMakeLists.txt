add_executable(Voxel14RegisterTest Voxel14RegisterTest.cpp)
target_link_libraries(Voxel14RegisterTest ti3dtof)

install(TARGETS
  Voxel14RegisterTest 
  RUNTIME
  DESTINATION bin
  COMPONENT ti3dtof_lib
)
