add_library(voxelvader SHARED
  VoxelVader.cpp
  VoxelVaderFactory.cpp
  ${COMMON_LIB_FILES}
)

set_voxel_library_output_directory(voxelvader)

generate_export_header(voxelvader
  EXPORT_FILE_NAME "SymbolExports.h"
  EXPORT_MACRO_NAME SYMBOL_EXPORT
)

target_link_libraries(voxelvader
  ti3dtof voxel ${COMMON_LIBS}
)

install(TARGETS voxelvader
  LIBRARY DESTINATION lib/voxel COMPONENT ti3dtof
  ARCHIVE DESTINATION lib/voxel COMPONENT ti3dtof
  RUNTIME DESTINATION lib/voxel COMPONENT ti3dtof
)

list(APPEND CONF_FILES
  boards/VoxelVader/VoxelVader.conf
  boards/VoxelVader/VoxelVaderLensOnly.conf
  boards/VoxelVader/VoxelVaderNormal.conf
)

set(CONF_FILES "${CONF_FILES}" PARENT_SCOPE)

list(APPEND UDEV_FILES
  boards/VoxelVader/72-voxelvader.rules
)

set(UDEV_FILES "${UDEV_FILES}" PARENT_SCOPE)

set_target_properties(voxelvader PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION})

