#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "shape_msgs::shape_msgs__rosidl_typesupport_introspection_c" for configuration "Release"
set_property(TARGET shape_msgs::shape_msgs__rosidl_typesupport_introspection_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(shape_msgs::shape_msgs__rosidl_typesupport_introspection_c PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libshape_msgs__rosidl_typesupport_introspection_c.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS shape_msgs::shape_msgs__rosidl_typesupport_introspection_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_shape_msgs::shape_msgs__rosidl_typesupport_introspection_c "${_IMPORT_PREFIX}/lib/libshape_msgs__rosidl_typesupport_introspection_c.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
