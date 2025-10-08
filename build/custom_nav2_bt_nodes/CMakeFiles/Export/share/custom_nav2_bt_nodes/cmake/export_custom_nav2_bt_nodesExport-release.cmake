#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "custom_nav2_bt_nodes::check_path_alignment_plugin" for configuration "Release"
set_property(TARGET custom_nav2_bt_nodes::check_path_alignment_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(custom_nav2_bt_nodes::check_path_alignment_plugin PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcheck_path_alignment_plugin.so"
  IMPORTED_SONAME_RELEASE "libcheck_path_alignment_plugin.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS custom_nav2_bt_nodes::check_path_alignment_plugin )
list(APPEND _IMPORT_CHECK_FILES_FOR_custom_nav2_bt_nodes::check_path_alignment_plugin "${_IMPORT_PREFIX}/lib/libcheck_path_alignment_plugin.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
