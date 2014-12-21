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
  
### Copy DML and firmware files to build directory
add_custom_command(TARGET ti3dtof PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/TI3DToF/OPT9220_0v27.fw $<TARGET_FILE_DIR:ti3dtof>)
                   
add_custom_command(TARGET ti3dtof PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/TI3DToF/OPT9220A.dml $<TARGET_FILE_DIR:ti3dtof>)
                   
                   
install(TARGETS ti3dtof
  LIBRARY DESTINATION lib/voxel
  ARCHIVE DESTINATION lib/voxel
  COMPONENT ti3dtof
)

install(FILES 
  OPT9220A.dml
  DESTINATION /etc/voxel/
  COMPONENT ti3dtof
)

install(FILES 
  OPT9220_0v27.fw
  DESTINATION /lib/firmware/voxel
  COMPONENT ti3dtof
)

install(FILES
  72-ti3dtof.rules
  DESTINATION /etc/udev/rules.d/
  COMPONENT ti3dtof
)

install(FILES
  ToFCamera.h
  ToFCameraFactory.h
  ToFHaddockCamera.h
  Voxel14Camera.h
  VoxelXUProgrammer.h
  DESTINATION include/voxel/ti3dtof
  COMPONENT ti3dtof-dev
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL ti3dtof)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Support libraries for Texas Instruments's ToF depth camera")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/debian/postrm;")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= 0.1)")
  create_cpack_config(libti3dtof 0.1)

  set(CPACK_COMPONENTS_ALL ti3dtof-dev)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Header files for ToF depth camera from Texas Instruments.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel-dev (>= 0.1)")
  create_cpack_config(libti3dtof-dev 0.1)
ELSEIF(WINDOWS)
  # Added NSIS related options here
ENDIF()