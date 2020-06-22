include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	SOLID3_INCLUDE_DIRS
	NAMES
	SOLID/SOLID.h
)

mark_as_advanced(SOLID3_INCLUDE_DIRS)

find_library(
	SOLID3_LIBRARY_DEBUG
	NAMES
	solid3d_d solid3_d solidd solidsd
)

find_library(
	SOLID3_LIBRARY_RELEASE
	NAMES
	solid3d solid3 solid solids
)

select_library_configurations(SOLID3)

find_package_handle_standard_args(
	SOLID3
	FOUND_VAR SOLID3_FOUND
	REQUIRED_VARS SOLID3_INCLUDE_DIRS SOLID3_LIBRARIES
)

if(SOLID3_FOUND AND NOT TARGET solid3::solid3)
	add_library(solid3::solid3 UNKNOWN IMPORTED)
	
	if(SOLID3_LIBRARY_RELEASE)
		set_property(TARGET solid3::solid3 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(solid3::solid3 PROPERTIES IMPORTED_LOCATION_RELEASE "${SOLID3_LIBRARY_RELEASE}")
	endif()
	
	if(SOLID3_LIBRARY_DEBUG)
		set_property(TARGET solid3::solid3 APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(solid3::solid3 PROPERTIES IMPORTED_LOCATION_DEBUG "${SOLID3_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		solid3::solid3 PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${SOLID3_INCLUDE_DIRS}"
	)
endif()
