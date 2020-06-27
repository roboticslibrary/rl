include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	OCTOMAP_INCLUDE_DIRS
	NAMES
	octomap/octomap.h
)

mark_as_advanced(OCTOMAP_INCLUDE_DIRS)

find_library(
	OCTOMAP_OCTOMAP_LIBRARY_DEBUG
	NAMES
	octomapd
)

find_library(
	OCTOMAP_OCTOMAP_LIBRARY_RELEASE
	NAMES
	octomap
)

select_library_configurations(OCTOMAP_OCTOMAP)

find_library(
	OCTOMAP_OCTOMATH_LIBRARY_DEBUG
	NAMES
	octomathd
)

find_library(
	OCTOMAP_OCTOMATH_LIBRARY_RELEASE
	NAMES
	octomath
)

select_library_configurations(OCTOMAP_OCTOMATH)

set(OCTOMAP_LIBRARIES ${OCTOMAP_OCTOMAP_LIBRARIES} ${OCTOMAP_OCTOMATH_LIBRARIES})

find_package_handle_standard_args(
	OctoMap
	FOUND_VAR OCTOMAP_FOUND
	REQUIRED_VARS OCTOMAP_INCLUDE_DIRS OCTOMAP_LIBRARIES
)

if(OCTOMAP_FOUND AND NOT TARGET octomap::octomap)
	add_library(octomap::octomap UNKNOWN IMPORTED)
	
	if(OCTOMAP_OCTOMAP_LIBRARY_RELEASE)
		set_property(TARGET octomap::octomap APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(octomap::octomap PROPERTIES IMPORTED_LOCATION_RELEASE "${OCTOMAP_OCTOMAP_LIBRARY_RELEASE}")
	endif()
	
	if(OCTOMAP_OCTOMAP_LIBRARY_DEBUG)
		set_property(TARGET octomap::octomap APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(octomap::octomap PROPERTIES IMPORTED_LOCATION_DEBUG "${OCTOMAP_OCTOMAP_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		octomap::octomap PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}"
	)
endif()

if(OCTOMAP_FOUND AND NOT TARGET octomap::octomath)
	add_library(octomap::octomath UNKNOWN IMPORTED)
	
	if(OCTOMAP_OCTOMATH_LIBRARY_RELEASE)
		set_property(TARGET octomap::octomath APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(octomap::octomath PROPERTIES IMPORTED_LOCATION_RELEASE "${OCTOMAP_OCTOMATH_LIBRARY_RELEASE}")
	endif()
	
	if(OCTOMAP_OCTOMATH_LIBRARY_DEBUG)
		set_property(TARGET octomap::octomath APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(octomap::octomath PROPERTIES IMPORTED_LOCATION_DEBUG "${OCTOMAP_OCTOMATH_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		octomap::octomath PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}"
	)
endif()
