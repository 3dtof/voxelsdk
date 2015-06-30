add_library(lipscamera SHARED
  LipsCamera.cpp
  LipsCameraFactory.cpp
)

generate_export_header(lipscamera
  EXPORT_FILE_NAME "SymbolExports.h"
  EXPORT_MACRO_NAME SYMBOL_EXPORT
)

target_link_libraries(lipscamera 
  ti3dtof voxel ${COMMON_LIBS}
)

install(TARGETS lipscamera
  LIBRARY DESTINATION lib/voxel COMPONENT ti3dtof
  ARCHIVE DESTINATION lib/voxel COMPONENT ti3dtof
  RUNTIME DESTINATION lib/voxel COMPONENT ti3dtof
)

list(APPEND CONF_FILES
  boards/LipsCamera/LipsCamera.conf
  boards/LipsCamera/LipsCameraNormal.conf
  boards/LipsCamera/LipsCameraTestMode.conf
  boards/LipsCamera/LipsCameraPhaseOffset.bin
)

set(CONF_FILES "${CONF_FILES}" PARENT_SCOPE)

list(APPEND UDEV_FILES
  boards/LipsCamera/72-lipscamera.rules
)

set(UDEV_FILES "${UDEV_FILES}" PARENT_SCOPE)

set_target_properties(lipscamera PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION}
                      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../.."
                      ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../.."
                      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../..")

