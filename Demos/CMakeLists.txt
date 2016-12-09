cmake_minimum_required(VERSION 2.8)
SET(VOXEL_DEMO_VERSION ${VOXEL_VERSION})
find_package(OpenCV REQUIRED)

add_definitions(-pthread -std=c++11 -fPIC -ffast-math)
add_executable(SimplePeopleTracking SimplePeopleTracking.cpp Horus.cpp TOFApp.cpp)
target_link_libraries(SimplePeopleTracking voxel ${OpenCV_LIBS} )

install(TARGETS
  SimplePeopleTracking
  RUNTIME
  DESTINATION bin
  COMPONENT demos
)

IF(LINUX)
  set(CPACK_COMPONENTS_ALL Demos)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Voxel sample applications")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libvoxel (>= ${VOXEL_VERSION})")
  create_cpack_config(libvoxel-demos ${VOXEL_DEMO_VERSION})
ENDIF()
