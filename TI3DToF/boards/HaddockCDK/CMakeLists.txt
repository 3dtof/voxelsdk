add_library(haddockcdk SHARED
  HaddockCDKCamera.cpp
  HaddockCDKCameraFactory.cpp
  ${COMMON_LIB_FILES}
)

set_voxel_library_output_directory(haddockcdk)

generate_export_header(haddockcdk
  EXPORT_FILE_NAME "SymbolExports.h"
  EXPORT_MACRO_NAME SYMBOL_EXPORT
)

target_link_libraries(haddockcdk 
  ti3dtof voxel ${COMMON_LIBS}
)

install(TARGETS haddockcdk
  LIBRARY DESTINATION lib/voxel COMPONENT ti3dtof
  ARCHIVE DESTINATION lib/voxel COMPONENT ti3dtof
  RUNTIME DESTINATION lib/voxel COMPONENT ti3dtof
)

list(APPEND CONF_FILES
  boards/HaddockCDK/HaddockCDKCamera.conf
  boards/HaddockCDK/HaddockCDKCameraNormal.conf
)

set(CONF_FILES "${CONF_FILES}" PARENT_SCOPE)

list(APPEND UDEV_FILES
  boards/HaddockCDK/72-HaddockCDK.rules
)

set(UDEV_FILES "${UDEV_FILES}" PARENT_SCOPE)

set_target_properties(haddockcdk PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION})

