#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "turtlelib::geometry" for configuration ""
set_property(TARGET turtlelib::geometry APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(turtlelib::geometry PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/geometry"
  )

list(APPEND _IMPORT_CHECK_TARGETS turtlelib::geometry )
list(APPEND _IMPORT_CHECK_FILES_FOR_turtlelib::geometry "${_IMPORT_PREFIX}/bin/geometry" )

# Import target "turtlelib::turtlelib" for configuration ""
set_property(TARGET turtlelib::turtlelib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(turtlelib::turtlelib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libturtlelib.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS turtlelib::turtlelib )
list(APPEND _IMPORT_CHECK_FILES_FOR_turtlelib::turtlelib "${_IMPORT_PREFIX}/lib/libturtlelib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
