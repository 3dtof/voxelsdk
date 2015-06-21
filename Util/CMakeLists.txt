SET(VOXEL_UTIL_VERSION ${VOXEL_VERSION})

add_executable(SerialNumber SerialNumber.cpp)
target_link_libraries(SerialNumber voxel)

install(TARGETS
  SerialNumber
  RUNTIME
  DESTINATION bin
  COMPONENT util
)
             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL util)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel library utility programs.")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION})")
  create_cpack_config(libvoxel-util ${VOXEL_UTIL_VERSION})
ENDIF()
