include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	CCD_INCLUDE_DIR
	NAMES ccd/ccd.h
)
find_library(
	CCD_LIBRARY_DEBUG
	NAMES ccdd
)
find_library(
	CCD_LIBRARY_RELEASE
	NAMES ccd
)
select_library_configurations(CCD)

set(CCD_INCLUDE_DIRS ${CCD_INCLUDE_DIR})
set(CCD_LIBRARIES ${CCD_LIBRARY})

find_package_handle_standard_args(
	ccd
	FOUND_VAR ccd_FOUND
	REQUIRED_VARS CCD_INCLUDE_DIR CCD_LIBRARY
)

if(ccd_FOUND AND NOT TARGET ccd::ccd)
	add_library(ccd::ccd UNKNOWN IMPORTED)
	if(CCD_LIBRARY_RELEASE)
		set_property(TARGET ccd::ccd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(ccd::ccd PROPERTIES IMPORTED_LOCATION_RELEASE "${CCD_LIBRARY_RELEASE}")
	endif()
	if(CCD_LIBRARY_DEBUG)
		set_property(TARGET ccd::ccd APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(ccd::ccd PROPERTIES IMPORTED_LOCATION_DEBUG "${CCD_LIBRARY_DEBUG}")
	endif()
	set_target_properties(ccd::ccd PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CCD_INCLUDE_DIRS}")
endif()

mark_as_advanced(CCD_INCLUDE_DIR)
