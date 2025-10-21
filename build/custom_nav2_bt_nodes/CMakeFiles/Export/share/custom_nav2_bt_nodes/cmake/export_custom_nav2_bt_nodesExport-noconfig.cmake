#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment" for configuration ""
set_property(TARGET custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libnav2_bt_nodes_check_path_alignment.so"
  IMPORTED_SONAME_NOCONFIG "libnav2_bt_nodes_check_path_alignment.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment )
list(APPEND _IMPORT_CHECK_FILES_FOR_custom_nav2_bt_nodes::nav2_bt_nodes_check_path_alignment "${_IMPORT_PREFIX}/lib/libnav2_bt_nodes_check_path_alignment.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
