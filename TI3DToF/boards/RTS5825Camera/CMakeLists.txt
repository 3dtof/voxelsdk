add_library(rts5825camera SHARED
  RTS5825Camera.cpp
  RTS5825CameraFactory.cpp
  ${COMMON_LIB_FILES}
)

generate_export_header(rts5825camera
  EXPORT_FILE_NAME "SymbolExports.h"
  EXPORT_MACRO_NAME SYMBOL_EXPORT
)

target_link_libraries(rts5825camera 
  ti3dtof voxel ${COMMON_LIBS}
)

install(TARGETS rts5825camera
  LIBRARY DESTINATION lib/voxel COMPONENT ti3dtof
  ARCHIVE DESTINATION lib/voxel COMPONENT ti3dtof
  RUNTIME DESTINATION lib/voxel COMPONENT ti3dtof
)

list(APPEND CONF_FILES
  boards/RTS5825Camera/RTS5825Camera.conf
  boards/RTS5825Camera/RTS5825CameraNormal.conf
  boards/RTS5825Camera/RTS5825CameraNoCalibration.conf
)

set(CONF_FILES "${CONF_FILES}" PARENT_SCOPE)

list(APPEND UDEV_FILES
  boards/RTS5825Camera/72-rts5825camera.rules
)

set(UDEV_FILES "${UDEV_FILES}" PARENT_SCOPE)

set_target_properties(rts5825camera PROPERTIES 
                      VERSION ${VOXEL_VERSION} 
                      SOVERSION ${VOXEL_ABI_VERSION}
                      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../.."
                      ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../.."
                      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../..")

