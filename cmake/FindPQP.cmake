include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	PQP_INCLUDE_DIRS
	NAMES
	PQP.h
)

mark_as_advanced(PQP_INCLUDE_DIRS)

find_library(
	PQP_LIBRARY_DEBUG
	NAMES
	PQPd
)

find_library(
	PQP_LIBRARY_RELEASE
	NAMES
	PQP
)

select_library_configurations(PQP)

find_package_handle_standard_args(
	PQP
	FOUND_VAR PQP_FOUND
	REQUIRED_VARS PQP_INCLUDE_DIRS PQP_LIBRARIES
)

if(PQP_FOUND AND NOT TARGET PQP::PQP)
	add_library(PQP::PQP UNKNOWN IMPORTED)
	
	if(PQP_LIBRARY_RELEASE)
		set_property(TARGET PQP::PQP APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(PQP::PQP PROPERTIES IMPORTED_LOCATION_RELEASE "${PQP_LIBRARY_RELEASE}")
	endif()
	
	if(PQP_LIBRARY_DEBUG)
		set_property(TARGET PQP::PQP APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(PQP::PQP PROPERTIES IMPORTED_LOCATION_DEBUG "${PQP_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		PQP::PQP PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${PQP_INCLUDE_DIRS}"
	)
endif()
