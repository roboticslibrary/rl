include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	NLOPT_INCLUDE_DIR
	NAMES nlopt.h
)
find_library(
	NLOPT_LIBRARY_DEBUG
	NAMES nloptd
)
find_library(
	NLOPT_LIBRARY_RELEASE
	NAMES nlopt
)
select_library_configurations(NLOPT)

set(NLOPT_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
set(NLOPT_LIBRARIES ${NLOPT_LIBRARY})

find_package_handle_standard_args(
	NLopt
	FOUND_VAR NLopt_FOUND
	REQUIRED_VARS NLOPT_INCLUDE_DIR NLOPT_LIBRARY
)

if(NLopt_FOUND AND NOT TARGET NLopt::nlopt)
	add_library(NLopt::nlopt UNKNOWN IMPORTED)
	if(NLOPT_LIBRARY_RELEASE)
		set_property(TARGET NLopt::nlopt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(NLopt::nlopt PROPERTIES IMPORTED_LOCATION_RELEASE "${NLOPT_LIBRARY_RELEASE}")
	endif()
	if(NLOPT_LIBRARY_DEBUG)
		set_property(TARGET NLopt::nlopt APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(NLopt::nlopt PROPERTIES IMPORTED_LOCATION_DEBUG "${NLOPT_LIBRARY_DEBUG}")
	endif()
	set_target_properties(NLopt::nlopt PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${NLOPT_INCLUDE_DIRS}")
endif()

mark_as_advanced(NLOPT_INCLUDE_DIR)
