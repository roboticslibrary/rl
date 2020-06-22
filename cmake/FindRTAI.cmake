include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	RTAI_INCLUDE_DIRS
	NAMES
	rtai_lxrt.h
	PATHS
	/usr/realtime/include
	PATH_SUFFIXES
	rtai
)

mark_as_advanced(RTAI_INCLUDE_DIRS)

find_library(
	RTAI_LIBRARY_DEBUG
	NAMES
	lxrtd
	PATHS
	/usr/realtime/lib
)

find_library(
	RTAI_LIBRARY_RELEASE
	NAMES
	lxrt
	PATHS
	/usr/realtime/lib
)

select_library_configurations(RTAI)

set(RTAI_LIBRARIES ${RTAI_LIBRARIES} rt)

find_package_handle_standard_args(
	RTAI
	FOUND_VAR RTAI_FOUND
	REQUIRED_VARS RTAI_INCLUDE_DIRS RTAI_LIBRARIES
)

if(RTAI_FOUND AND NOT TARGET RTAI::lxrt)
	add_library(RTAI::lxrt UNKNOWN IMPORTED)
	
	if(RTAI_LIBRARY_RELEASE)
		set_property(TARGET RTAI::lxrt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(RTAI::lxrt PROPERTIES IMPORTED_LOCATION_RELEASE "${RTAI_LIBRARY_RELEASE}")
	endif()
	
	if(RTAI_LIBRARY_DEBUG)
		set_property(TARGET RTAI::lxrt APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(RTAI::lxrt PROPERTIES IMPORTED_LOCATION_DEBUG "${RTAI_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		RTAI::lxrt PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${RTAI_INCLUDE_DIRS}"
	)
endif()
