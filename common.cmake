FUNCTION(create_cmake_config package targets component build_include_dir)
# Add all targets to the build-tree export set
export(TARGETS ${targets}
  FILE "${PROJECT_BINARY_DIR}/${package}Targets.cmake")
  
string(TOUPPER ${package} UPACKAGE)
 
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${package})

if(WINDOWS)
  set(INSTALL_CMAKE_DIR CMake)
elseif(LINUX)
  set(INSTALL_CMAKE_DIR lib/cmake/${package})
endif()
 
# Create the FooBarConfig.cmake and FooBarConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR} "${CMAKE_INSTALL_PREFIX}/include/voxel")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/${build_include_dir}" "${PROJECT_BINARY_DIR}/${build_include_dir}")
configure_file(${package}Config.cmake.in "${PROJECT_BINARY_DIR}/${package}Config.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${${UPACKAGE}_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${package}Config.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${package}Config.cmake" @ONLY)
# ... for both
configure_file(${package}ConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake" @ONLY)
 
# Install the FooBarConfig.cmake and FooBarConfigVersion.cmake
install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${package}Config.cmake"
    "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
  DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT ${component})
  
# Install the export set for use with the install-tree
install(EXPORT ${package}Targets DESTINATION
  ${INSTALL_CMAKE_DIR} COMPONENT ${component})
 
ENDFUNCTION()