FUNCTION(create_cmake_config package targets component)
# Add all targets to the build-tree export set
export(TARGETS ${targets}
  FILE "${PROJECT_BINARY_DIR}/${package}Targets.cmake")
 
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
#export(PACKAGE FooBar)
 
# Create the FooBarConfig.cmake and FooBarConfigVersion files
set(REL_INCLUDE_DIR "cmake include")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${package}Config.cmake.in "${PROJECT_BINARY_DIR}/${package}Config.cmake" @ONLY)
# ... for the install tree
#set(CONF_INCLUDE_DIRS "\${${package}_CMAKE_DIR}/${REL_INCLUDE_DIR}")
#configure_file(${package}Config.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${package}Config.cmake" @ONLY)
# ... for both
configure_file(${package}ConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake" @ONLY)
 
if(WINDOWS)

# Install the FooBarConfig.cmake and FooBarConfigVersion.cmake
install(FILES
    "${PROJECT_BINARY_DIR}/${package}Config.cmake"
    "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
  DESTINATION lib/cmake COMPONENT ${component})
  
# Install the export set for use with the install-tree
install(EXPORT ${package}Targets DESTINATION
  lib/cmake COMPONENT ${component})
  
elseif(LINUX)
# Install the FooBarConfig.cmake and FooBarConfigVersion.cmake
install(FILES
    "${PROJECT_BINARY_DIR}/${package}Config.cmake"
    "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
  DESTINATION lib/${package} COMPONENT ${component})
  
# Install the export set for use with the install-tree
install(EXPORT ${package}Targets DESTINATION
  lib/${package} COMPONENT ${component})
endif()
 
ENDFUNCTION()