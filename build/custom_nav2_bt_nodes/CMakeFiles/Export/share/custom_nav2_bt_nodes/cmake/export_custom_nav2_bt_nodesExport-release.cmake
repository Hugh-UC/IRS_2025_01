#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment" for configuration "Release"
set_property(TARGET custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libnav2_bt_nodes_check_path_alignment.so"
  IMPORTED_SONAME_RELEASE "libnav2_bt_nodes_check_path_alignment.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment )
list(APPEND _IMPORT_CHECK_FILES_FOR_custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment "${_IMPORT_PREFIX}/lib/libnav2_bt_nodes_check_path_alignment.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
