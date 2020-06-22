include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	libdc1394_INCLUDE_DIRS
	NAMES
	dc1394/dc1394.h
)

mark_as_advanced(libdc1394_INCLUDE_DIRS)

find_library(
	libdc1394_LIBRARY_RELEASE
	NAMES
	dc1394
)

mark_as_advanced(libdc1394_LIBRARY_RELEASE)

select_library_configurations(libdc1394)

find_package_handle_standard_args(
	libdc1394
	FOUND_VAR libdc1394_FOUND
	REQUIRED_VARS libdc1394_INCLUDE_DIRS libdc1394_LIBRARIES
)

if(libdc1394_FOUND AND NOT TARGET libdc1394::libdc1394)
	add_library(libdc1394::libdc1394 UNKNOWN IMPORTED)
	
	if(libdc1394_LIBRARY_RELEASE)
		set_property(TARGET libdc1394::libdc1394 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(libdc1394::libdc1394 PROPERTIES IMPORTED_LOCATION_RELEASE "${libdc1394_LIBRARY_RELEASE}")
	endif()
	
	set_target_properties(
		libdc1394::libdc1394 PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${libdc1394_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${libdc1394_INCLUDE_DIRS}"
	)
endif()
