add_library(voxel14 SHARED
  Voxel14Camera.cpp
  Voxel14CameraFactory.cpp
)

generate_export_header(voxel14
  EXPORT_FILE_NAME "SymbolExports.h"
  EXPORT_MACRO_NAME SYMBOL_EXPORT
)

target_link_libraries(voxel14 
  ti3dtof voxel ${COMMON_LIBS}
)

install(TARGETS voxel14
  LIBRARY DESTINATION lib/voxel COMPONENT ti3dtof
  ARCHIVE DESTINATION lib/voxel COMPONENT ti3dtof
  RUNTIME DESTINATION lib/voxel COMPONENT ti3dtof
)

list(APPEND CONF_FILES
  boards/Voxel14/Voxel14Camera.conf
  boards/Voxel14/Voxel14CameraNormal.conf
  boards/Voxel14/Voxel14CameraTestMode.conf
  boards/Voxel14/Voxel14CameraPhaseOffset.bin
)

set(CONF_FILES "${CONF_FILES}" PARENT_SCOPE)

list(APPEND UDEV_FILES
  boards/Voxel14/72-Voxel14.rules
)

set(UDEV_FILES "${UDEV_FILES}" PARENT_SCOPE)

set_target_properties(voxel14 PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION}
                      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../.."
                      ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../.."
                      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../..")

